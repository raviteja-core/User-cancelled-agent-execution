#pragma once

#include "core/fsm/state_machine.hpp"
#include "core/motion/planner.hpp"
#include "core/protocol/packet.hpp"
#include <cstdint>

namespace papercutter::safety {

class SafetySupervisor {
public:
    explicit SafetySupervisor(uint32_t heartbeat_timeout_ms = 200) noexcept;

    // Evaluates all safety interlocks for the current 1 ms tick.
    // If any safety rule is violated, immediately commands FSM and motion planners to halt.
    void tick_1ms(
        fsm::StateMachine& fsm,
        motion::TrapezoidalPlanner& back_gauge,
        motion::BladePlanner& blade,
        uint16_t recent_crc_errors,
        bool link_connected
    ) noexcept;

    // Heartbeat watchdog pet (called whenever valid packet arrives from HMI)
    void pet_watchdog() noexcept;

    // Checkers
    [[nodiscard]] bool is_estop_active() const noexcept { return estop_active_; }
    [[nodiscard]] bool is_watchdog_tripped() const noexcept { return watchdog_tripped_; }
    [[nodiscard]] uint32_t watchdog_elapsed_ms() const noexcept { return watchdog_timer_ms_; }

private:
    uint32_t heartbeat_timeout_ms_{200};
    uint32_t watchdog_timer_ms_{0};
    bool watchdog_tripped_{false};
    bool estop_active_{false};

    // CRC storm detector
    uint16_t last_crc_count_{0};
    uint16_t crc_window_errors_{0};
    uint32_t crc_window_timer_ms_{0};
};

} // namespace papercutter::safety
