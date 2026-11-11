#include "core/sim/sim_hardware.hpp"

namespace papercutter::sim {

SimulatedHardware::SimulatedHardware(uint32_t blade_stroke_steps) noexcept
    : blade_stroke_steps_(blade_stroke_steps) {}

void SimulatedHardware::set_clamp_engaged(bool engage) noexcept {
    clamp_commanded_ = engage;
    if (!engage) {
        clamp_timer_ms_ = 0;
        clamp_switch_closed_ = false;
    }
}

void SimulatedHardware::inject_paper_jam(bool jam_active) noexcept {
    paper_jam_injected_ = jam_active;
    if (jam_active) {
        feed_sensor_active_ = false;
    }
}

void SimulatedHardware::inject_premature_blade_limit(bool trigger) noexcept {
    premature_limit_injected_ = trigger;
}

void SimulatedHardware::inject_interlock_trip(bool trip) noexcept {
    interlock_tripped_ = trip;
}

uint8_t SimulatedHardware::limit_switch_bitmask() const noexcept {
    uint8_t mask = 0;
    if (blade_tdc_switch_) mask |= static_cast<uint8_t>(protocol::LimitSwitchBit::BLADE_TDC);
    if (blade_bdc_switch_) mask |= static_cast<uint8_t>(protocol::LimitSwitchBit::BLADE_BDC);
    if (clamp_switch_closed_) mask |= static_cast<uint8_t>(protocol::LimitSwitchBit::CLAMP_SWITCH);
    if (!interlock_tripped_) mask |= static_cast<uint8_t>(protocol::LimitSwitchBit::SAFETY_GUARD);
    return mask;
}

void SimulatedHardware::tick(
    uint32_t delta_ms,
    fsm::StateMachine& fsm,
    motion::TrapezoidalPlanner& back_gauge,
    motion::BladePlanner& blade
) noexcept {
    auto current_state = fsm.state();

    // Check premature limit switch injection
    if (premature_limit_injected_ && current_state != protocol::State::CUTTING) {
        fsm.handle_event(fsm::Event::PREMATURE_LIMIT);
        return;
    }

    // Check physical safety guard / interlock
    if (interlock_tripped_) {
        fsm.force_estop();
        return;
    }

    // Update FSM timer (handles jam timeout)
    fsm.tick(delta_ms);

    if (current_state == protocol::State::FEEDING) {
        // Back-gauge motion active
        if (back_gauge.is_moving()) {
            back_gauge.tick_1ms();
        }

        if (!back_gauge.is_moving()) {
            if (paper_jam_injected_) {
                feed_sensor_active_ = false;
                // Will time out in FSM tick after 3000 ms!
            } else {
                feed_sensor_active_ = true;
                fsm.context().feed_sensor_active = true;
                fsm.handle_event(fsm::Event::FEED_POS_REACHED);
            }
        }
    } else if (current_state == protocol::State::CLAMPING) {
        set_clamp_engaged(true);
        clamp_timer_ms_ += delta_ms;
        if (clamp_timer_ms_ >= CLAMP_ENGAGE_DELAY_MS) {
            clamp_switch_closed_ = true;
            fsm.context().clamp_closed = true;
            fsm.handle_event(fsm::Event::CLAMP_CONFIRMED);
            blade.start_cycle();
        }
    } else if (current_state == protocol::State::CUTTING) {
        bool cycle_complete = blade.tick_1ms();
        int32_t blade_pos = blade.position_steps();

        blade_tdc_switch_ = (blade_pos <= 50);
        blade_bdc_switch_ = (blade_pos >= static_cast<int32_t>(blade_stroke_steps_ - 50));

        fsm.context().blade_tdc_active = blade_tdc_switch_;
        fsm.context().blade_bdc_active = blade_bdc_switch_;

        if (cycle_complete) {
            blade_tdc_switch_ = true;
            blade_bdc_switch_ = false;
            set_clamp_engaged(false);
            fsm.context().clamp_closed = false;
            fsm.context().blade_tdc_active = true;
            fsm.handle_event(fsm::Event::CUT_COMPLETE);

            // If transitioned to next cut in FEEDING, plan back-gauge move
            if (fsm.state() == protocol::State::FEEDING) {
                uint8_t next_idx = fsm.context().current_cut;
                uint32_t target_pos = fsm.context().cut_positions[next_idx];
                back_gauge.plan_move(static_cast<int32_t>(target_pos));
            }
        }
    } else {
        // IDLE or ERROR states
        if (clamp_commanded_ && current_state != protocol::State::CLAMPING && current_state != protocol::State::CUTTING) {
            set_clamp_engaged(false);
        }
    }
}

protocol::TelemetryPayload SimulatedHardware::generate_telemetry(
    const fsm::StateMachine& fsm,
    const motion::TrapezoidalPlanner& back_gauge,
    const motion::BladePlanner& blade,
    uint16_t crc_errors,
    uint16_t dropped_packets,
    uint16_t timeout_errors
) const noexcept {
    protocol::TelemetryPayload t{};
    t.state = static_cast<uint8_t>(fsm.state());
    t.fault_code = static_cast<uint16_t>(fsm.context().latched_fault);
    t.back_gauge_pos = back_gauge.current_position_steps();
    t.blade_pos = blade.position_steps();
    t.feed_sensor = feed_sensor_active_ ? 1 : 0;
    t.limit_switches = limit_switch_bitmask();
    t.cut_index = fsm.context().current_cut;
    t.crc_errors = crc_errors;
    t.dropped_packets = dropped_packets;
    t.timeout_errors = timeout_errors;
    t.clamp_engaged = clamp_switch_closed_ ? 1 : 0;
    t.reserved = 0;
    return t;
}

} // namespace papercutter::sim
