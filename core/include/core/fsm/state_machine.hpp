#pragma once

#include "core/protocol/packet.hpp"
#include <cstddef>
#include <cstdint>
#include <array>

namespace papercutter::fsm {

enum class Event : uint8_t {
    START = 0,
    FEED_POS_REACHED = 1,
    FEED_SENSOR_CHANGED = 2,
    JAM_TIMEOUT = 3,
    CLAMP_CONFIRMED = 4,
    CUT_COMPLETE = 5,
    ESTOP = 6,
    HEARTBEAT_TIMEOUT = 7,
    PREMATURE_LIMIT = 8,
    RESET_FAULT = 9,
    ABORT = 10,
    EVENT_COUNT
};

enum class TransitionResult : uint8_t {
    SUCCESS = 0,
    ILLEGAL_TRANSITION = 1,
    GUARD_FAILED = 2
};

struct MachineContext {
    bool job_loaded{false};
    uint16_t job_id{0};
    uint8_t total_cuts{0};
    uint8_t current_cut{0};
    std::array<uint32_t, 64> cut_positions{}; // 0.01 mm units

    bool feed_sensor_active{false};
    bool clamp_closed{false};
    bool blade_tdc_active{true};
    bool blade_bdc_active{false};
    bool estop_asserted{false};
    bool interlocks_safe{true};

    // Output flags commanded by actions
    bool back_gauge_moving{false};
    bool clamp_actuating{false};
    bool blade_moving{false};
    bool motors_halted{true};

    protocol::FaultCode latched_fault{protocol::FaultCode::NONE};
};

using GuardFunc = bool (*)(const MachineContext&) noexcept;
using ActionFunc = void (*)(MachineContext&) noexcept;

struct TransitionEntry {
    protocol::State from_state;
    Event event;
    GuardFunc guard;
    protocol::State to_state;
    ActionFunc action;
};

class StateMachine {
public:
    constexpr StateMachine() noexcept : state_(protocol::State::IDLE) {}

    // Dispatches an event according to the transition table.
    TransitionResult handle_event(Event ev) noexcept;

    // 1 ms tick for timer evaluation (e.g. jam timeout detection)
    void tick(uint32_t delta_ms) noexcept;

    [[nodiscard]] constexpr protocol::State state() const noexcept { return state_; }
    [[nodiscard]] constexpr const MachineContext& context() const noexcept { return ctx_; }
    MachineContext& context() noexcept { return ctx_; }

    void force_estop() noexcept;

private:
    protocol::State state_{protocol::State::IDLE};
    MachineContext ctx_{};
    uint32_t jam_timer_ms_{0};

    TransitionResult execute_transition(const TransitionEntry& entry) noexcept;
};

} // namespace papercutter::fsm
