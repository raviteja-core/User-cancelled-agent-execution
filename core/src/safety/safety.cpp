#include "core/safety/safety.hpp"

namespace papercutter::safety {

SafetySupervisor::SafetySupervisor(uint32_t heartbeat_timeout_ms) noexcept
    : heartbeat_timeout_ms_(heartbeat_timeout_ms) {}

void SafetySupervisor::pet_watchdog() noexcept {
    watchdog_timer_ms_ = 0;
    watchdog_tripped_ = false;
}

void SafetySupervisor::tick_1ms(
    fsm::StateMachine& fsm,
    motion::TrapezoidalPlanner& back_gauge,
    motion::BladePlanner& blade,
    uint16_t recent_crc_errors,
    bool link_connected
) noexcept {
    auto current_state = fsm.state();

    // 1. Check physical E-stop assertion
    if (fsm.context().estop_asserted) {
        estop_active_ = true;
        if (current_state != protocol::State::ERROR_FAULT) {
            fsm.handle_event(fsm::Event::ESTOP);
        }
        back_gauge.emergency_stop();
        blade.emergency_stop();
        return;
    }

    // 2. Communication watchdog (only enforced once connected)
    if (link_connected) {
        watchdog_timer_ms_ += 1;
        if (watchdog_timer_ms_ >= heartbeat_timeout_ms_) {
            watchdog_tripped_ = true;
            if (current_state != protocol::State::ERROR_FAULT && current_state != protocol::State::ERROR_JAM) {
                fsm.handle_event(fsm::Event::HEARTBEAT_TIMEOUT);
                back_gauge.emergency_stop();
                blade.emergency_stop();
                return;
            }
        }
    }

    // 3. CRC Storm Detection: > 5 CRC errors in 100 ms
    crc_window_timer_ms_ += 1;
    if (recent_crc_errors > last_crc_count_) {
        crc_window_errors_ += (recent_crc_errors - last_crc_count_);
        last_crc_count_ = recent_crc_errors;
    }
    if (crc_window_timer_ms_ >= 100) {
        if (crc_window_errors_ >= 5) {
            // Latch CRC Storm fault
            fsm.context().latched_fault = protocol::FaultCode::CRC_STORM;
            fsm.handle_event(fsm::Event::ESTOP);
            back_gauge.emergency_stop();
            blade.emergency_stop();
            return;
        }
        crc_window_timer_ms_ = 0;
        crc_window_errors_ = 0;
    }

    // 4. Motion boundary overrun detection
    int32_t bg_pos = back_gauge.current_position_steps();
    if (bg_pos < 0 || bg_pos > 100000) { // 0 to 1000 mm max allowable
        fsm.context().latched_fault = protocol::FaultCode::MOTION_OVERRUN;
        fsm.handle_event(fsm::Event::ESTOP);
        back_gauge.emergency_stop();
        blade.emergency_stop();
        return;
    }

    // 5. Premature blade limit detection
    if (current_state != protocol::State::CUTTING && !fsm.context().blade_tdc_active) {
        fsm.handle_event(fsm::Event::PREMATURE_LIMIT);
        back_gauge.emergency_stop();
        blade.emergency_stop();
        return;
    }
}

} // namespace papercutter::safety
