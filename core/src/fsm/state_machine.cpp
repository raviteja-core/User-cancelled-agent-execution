#include "core/fsm/state_machine.hpp"

namespace papercutter::fsm {

namespace {

// Guards
bool guard_always_true(const MachineContext&) noexcept {
    return true;
}

bool guard_start(const MachineContext& ctx) noexcept {
    return ctx.job_loaded && ctx.total_cuts > 0 && ctx.interlocks_safe && !ctx.estop_asserted;
}

bool guard_feed_pos_reached(const MachineContext& ctx) noexcept {
    return ctx.feed_sensor_active;
}

bool guard_clamp_confirmed(const MachineContext& ctx) noexcept {
    return ctx.clamp_closed;
}

bool guard_cut_complete_next(const MachineContext& ctx) noexcept {
    return (ctx.current_cut + 1) < ctx.total_cuts;
}

bool guard_cut_complete_done(const MachineContext& ctx) noexcept {
    return (ctx.current_cut + 1) >= ctx.total_cuts;
}

bool guard_reset_fault(const MachineContext& ctx) noexcept {
    return !ctx.estop_asserted && ctx.interlocks_safe && ctx.blade_tdc_active;
}

// Actions
void action_start_feed(MachineContext& ctx) noexcept {
    ctx.back_gauge_moving = true;
    ctx.motors_halted = false;
}

void action_start_clamp(MachineContext& ctx) noexcept {
    ctx.back_gauge_moving = false;
    ctx.clamp_actuating = true;
}

void action_start_cut(MachineContext& ctx) noexcept {
    ctx.clamp_actuating = false;
    ctx.blade_moving = true;
}

void action_next_cut(MachineContext& ctx) noexcept {
    ctx.blade_moving = false;
    ctx.current_cut++;
    ctx.back_gauge_moving = true;
}

void action_finish_job(MachineContext& ctx) noexcept {
    ctx.blade_moving = false;
    ctx.clamp_actuating = false;
    ctx.back_gauge_moving = false;
    ctx.job_loaded = false;
    ctx.motors_halted = true;
}

void action_jam_fault(MachineContext& ctx) noexcept {
    ctx.back_gauge_moving = false;
    ctx.clamp_actuating = false;
    ctx.blade_moving = false;
    ctx.motors_halted = true;
    ctx.latched_fault = protocol::FaultCode::PAPER_JAM;
}

void action_estop_fault(MachineContext& ctx) noexcept {
    ctx.back_gauge_moving = false;
    ctx.clamp_actuating = false;
    ctx.blade_moving = false;
    ctx.motors_halted = true;
    if (ctx.latched_fault == protocol::FaultCode::NONE) {
        ctx.latched_fault = protocol::FaultCode::ESTOP;
    }
}

void action_comm_fault(MachineContext& ctx) noexcept {
    ctx.back_gauge_moving = false;
    ctx.clamp_actuating = false;
    ctx.blade_moving = false;
    ctx.motors_halted = true;
    ctx.latched_fault = protocol::FaultCode::COMM_TIMEOUT;
}

void action_limit_fault(MachineContext& ctx) noexcept {
    ctx.back_gauge_moving = false;
    ctx.clamp_actuating = false;
    ctx.blade_moving = false;
    ctx.motors_halted = true;
    ctx.latched_fault = protocol::FaultCode::PREMATURE_LIMIT;
}

void action_reset_fault(MachineContext& ctx) noexcept {
    ctx.latched_fault = protocol::FaultCode::NONE;
    ctx.current_cut = 0;
    ctx.motors_halted = true;
}

void action_abort(MachineContext& ctx) noexcept {
    ctx.back_gauge_moving = false;
    ctx.clamp_actuating = false;
    ctx.blade_moving = false;
    ctx.motors_halted = true;
    ctx.job_loaded = false;
}

// Transition Table
constexpr TransitionEntry TRANSITION_TABLE[] = {
    // Normal operation sequence
    { protocol::State::IDLE, Event::START, guard_start, protocol::State::FEEDING, action_start_feed },
    { protocol::State::FEEDING, Event::FEED_POS_REACHED, guard_feed_pos_reached, protocol::State::CLAMPING, action_start_clamp },
    { protocol::State::CLAMPING, Event::CLAMP_CONFIRMED, guard_clamp_confirmed, protocol::State::CUTTING, action_start_cut },
    { protocol::State::CUTTING, Event::CUT_COMPLETE, guard_cut_complete_next, protocol::State::FEEDING, action_next_cut },
    { protocol::State::CUTTING, Event::CUT_COMPLETE, guard_cut_complete_done, protocol::State::IDLE, action_finish_job },

    // Jam timeout during feeding
    { protocol::State::FEEDING, Event::JAM_TIMEOUT, guard_always_true, protocol::State::ERROR_JAM, action_jam_fault },

    // E-stop from any state
    { protocol::State::IDLE, Event::ESTOP, guard_always_true, protocol::State::ERROR_FAULT, action_estop_fault },
    { protocol::State::FEEDING, Event::ESTOP, guard_always_true, protocol::State::ERROR_FAULT, action_estop_fault },
    { protocol::State::CLAMPING, Event::ESTOP, guard_always_true, protocol::State::ERROR_FAULT, action_estop_fault },
    { protocol::State::CUTTING, Event::ESTOP, guard_always_true, protocol::State::ERROR_FAULT, action_estop_fault },
    { protocol::State::ERROR_JAM, Event::ESTOP, guard_always_true, protocol::State::ERROR_FAULT, action_estop_fault },
    { protocol::State::ERROR_FAULT, Event::ESTOP, guard_always_true, protocol::State::ERROR_FAULT, action_estop_fault },

    // Heartbeat timeout from any operational state
    { protocol::State::IDLE, Event::HEARTBEAT_TIMEOUT, guard_always_true, protocol::State::ERROR_FAULT, action_comm_fault },
    { protocol::State::FEEDING, Event::HEARTBEAT_TIMEOUT, guard_always_true, protocol::State::ERROR_FAULT, action_comm_fault },
    { protocol::State::CLAMPING, Event::HEARTBEAT_TIMEOUT, guard_always_true, protocol::State::ERROR_FAULT, action_comm_fault },
    { protocol::State::CUTTING, Event::HEARTBEAT_TIMEOUT, guard_always_true, protocol::State::ERROR_FAULT, action_comm_fault },

    // Premature blade limit switch
    { protocol::State::IDLE, Event::PREMATURE_LIMIT, guard_always_true, protocol::State::ERROR_FAULT, action_limit_fault },
    { protocol::State::FEEDING, Event::PREMATURE_LIMIT, guard_always_true, protocol::State::ERROR_FAULT, action_limit_fault },
    { protocol::State::CLAMPING, Event::PREMATURE_LIMIT, guard_always_true, protocol::State::ERROR_FAULT, action_limit_fault },

    // Reset fault (from error states only)
    { protocol::State::ERROR_JAM, Event::RESET_FAULT, guard_reset_fault, protocol::State::IDLE, action_reset_fault },
    { protocol::State::ERROR_FAULT, Event::RESET_FAULT, guard_reset_fault, protocol::State::IDLE, action_reset_fault },

    // Abort
    { protocol::State::FEEDING, Event::ABORT, guard_always_true, protocol::State::IDLE, action_abort },
    { protocol::State::CLAMPING, Event::ABORT, guard_always_true, protocol::State::IDLE, action_abort },
    { protocol::State::CUTTING, Event::ABORT, guard_always_true, protocol::State::IDLE, action_abort },
};

constexpr size_t NUM_TRANSITIONS = sizeof(TRANSITION_TABLE) / sizeof(TRANSITION_TABLE[0]);

} // namespace

TransitionResult StateMachine::execute_transition(const TransitionEntry& entry) noexcept {
    if (entry.guard != nullptr && !entry.guard(ctx_)) {
        return TransitionResult::GUARD_FAILED;
    }

    if (entry.action != nullptr) {
        entry.action(ctx_);
    }

    state_ = entry.to_state;
    if (state_ == protocol::State::FEEDING) {
        jam_timer_ms_ = 0;
    }
    return TransitionResult::SUCCESS;
}

TransitionResult StateMachine::handle_event(Event ev) noexcept {
    bool match_found = false;
    for (size_t i = 0; i < NUM_TRANSITIONS; ++i) {
        const auto& entry = TRANSITION_TABLE[i];
        if (entry.from_state == state_ && entry.event == ev) {
            match_found = true;
            TransitionResult res = execute_transition(entry);
            if (res == TransitionResult::SUCCESS) {
                return TransitionResult::SUCCESS;
            }
        }
    }

    return match_found ? TransitionResult::GUARD_FAILED : TransitionResult::ILLEGAL_TRANSITION;
}

void StateMachine::tick(uint32_t delta_ms) noexcept {
    if (state_ == protocol::State::FEEDING) {
        jam_timer_ms_ += delta_ms;
        // 3.0 s (3000 ms) jam timeout
        if (jam_timer_ms_ >= 3000 && !ctx_.feed_sensor_active) {
            handle_event(Event::JAM_TIMEOUT);
        }
    }
}

void StateMachine::force_estop() noexcept {
    ctx_.estop_asserted = true;
    handle_event(Event::ESTOP);
}

} // namespace papercutter::fsm
