#include "core/motion/planner.hpp"
#include <algorithm>

namespace papercutter::motion {

namespace {

// Deterministic integer square root (standard digit-by-digit or Newton's method)
uint32_t isqrt(uint64_t val) noexcept {
    if (val == 0) return 0;
    uint64_t x = val;
    uint64_t y = (x + 1) / 2;
    while (y < x) {
        x = y;
        y = (x + val / x) / 2;
    }
    return static_cast<uint32_t>(x);
}

} // namespace

TrapezoidalPlanner::TrapezoidalPlanner(const MotionLimits& limits) noexcept
    : limits_(limits) {}

void TrapezoidalPlanner::set_limits(const MotionLimits& limits) noexcept {
    limits_ = limits;
}

void TrapezoidalPlanner::set_current_position(int32_t pos_steps) noexcept {
    current_position_ = pos_steps;
    target_position_ = pos_steps;
    current_velocity_ = 0;
    peak_velocity_ = 0;
    direction_ = 0;
    state_ = MotionState::IDLE;
    step_remainder_ = 0;
}

bool TrapezoidalPlanner::plan_move(int32_t target_position_steps) noexcept {
    if (target_position_steps < limits_.min_position_steps ||
        target_position_steps > limits_.max_position_steps) {
        return false;
    }

    if (target_position_steps == current_position_) {
        target_position_ = target_position_steps;
        state_ = MotionState::COMPLETE;
        direction_ = 0;
        current_velocity_ = 0;
        return true;
    }

    target_position_ = target_position_steps;
    direction_ = (target_position_ > current_position_) ? 1 : -1;
    total_distance_steps_ = static_cast<uint32_t>(std::abs(target_position_ - current_position_));
    steps_moved_in_profile_ = 0;
    step_remainder_ = 0;
    peak_velocity_ = 0;
    stopping_distance_steps_ = 0;

    uint32_t a = limits_.max_accel_steps_s2;
    uint32_t v_max = limits_.max_velocity_steps_s;
    if (a == 0) a = 1;

    // s_accel = v_max^2 / (2 * a)
    uint64_t s_req = (static_cast<uint64_t>(v_max) * v_max) / (2ULL * a);

    if (2 * s_req > total_distance_steps_) {
        // Triangular profile: peak velocity not reached
        // v_peak = sqrt(a * total_distance)
        uint64_t a_dist = static_cast<uint64_t>(a) * total_distance_steps_;
        cruise_velocity_ = isqrt(a_dist);
        if (cruise_velocity_ > v_max) cruise_velocity_ = v_max;
        accel_steps_ = total_distance_steps_ / 2;
        decel_steps_ = total_distance_steps_ - accel_steps_;
    } else {
        // Trapezoidal profile
        cruise_velocity_ = v_max;
        accel_steps_ = static_cast<uint32_t>(s_req);
        decel_steps_ = accel_steps_;
    }

    current_velocity_ = 0;
    state_ = MotionState::ACCELERATING;
    return true;
}

void TrapezoidalPlanner::emergency_stop() noexcept {
    if (state_ == MotionState::IDLE || state_ == MotionState::COMPLETE) {
        return;
    }
    state_ = MotionState::EMERGENCY_STOPPING;
    emergency_stop_start_pos_ = current_position_;
}

void TrapezoidalPlanner::hard_stop() noexcept {
    current_velocity_ = 0;
    step_interval_us_ = 0;
    step_remainder_ = 0;
    direction_ = 0;
    state_ = MotionState::COMPLETE;
}

uint32_t TrapezoidalPlanner::tick_1ms() noexcept {
    if (state_ == MotionState::IDLE || state_ == MotionState::COMPLETE) {
        current_velocity_ = 0;
        step_interval_us_ = 0;
        return 0;
    }

    uint32_t remaining_steps = static_cast<uint32_t>(std::abs(target_position_ - current_position_));
    if (remaining_steps == 0) {
        state_ = MotionState::COMPLETE;
        current_velocity_ = 0;
        step_interval_us_ = 0;
        return 0;
    }

    uint32_t a = limits_.max_accel_steps_s2;
    if (a == 0) a = 1;

    uint32_t v_curr = current_velocity_;
    uint32_t v_next = v_curr;

    if (state_ == MotionState::EMERGENCY_STOPPING) {
        uint32_t dv_em = limits_.emergency_decel_steps_s2 / 1000;
        if (dv_em == 0) dv_em = 1;
        if (v_curr > dv_em) {
            v_next = v_curr - dv_em;
        } else {
            v_next = 0;
        }
    } else {
        uint32_t dv = a / 1000;
        if (dv == 0) dv = 1;

        // Exact discrete stopping distance required to decelerate from v_curr to 0 at rate dv/ms
        uint64_t stop_dist = (static_cast<uint64_t>(v_curr) * (v_curr + dv)) / (2ULL * a);

        if (remaining_steps <= stop_dist || state_ == MotionState::DECELERATING) {
            state_ = MotionState::DECELERATING;
            if (v_curr > dv) {
                v_next = v_curr - dv;
            } else {
                v_next = 0;
            }
        } else if (state_ == MotionState::ACCELERATING) {
            v_next = v_curr + dv;
            if (v_next >= cruise_velocity_) {
                v_next = cruise_velocity_;
                state_ = MotionState::CRUISING;
            }
        } else if (state_ == MotionState::CRUISING) {
            v_next = cruise_velocity_;
        }
    }

    if (v_next > peak_velocity_) {
        peak_velocity_ = v_next;
    }

    // Average velocity for the 1 ms interval
    uint32_t v_avg = (v_curr + v_next) / 2;
    if (v_avg == 0 && (state_ != MotionState::EMERGENCY_STOPPING)) {
        v_avg = 1; // Ensure progress to avoid deadlocks at start
    }

    uint32_t step_accum = v_avg + step_remainder_;
    uint32_t steps = step_accum / 1000;
    step_remainder_ = step_accum % 1000;

    if (steps > remaining_steps) {
        steps = remaining_steps;
        v_next = 0;
    }

    current_position_ += direction_ * static_cast<int32_t>(steps);
    steps_moved_in_profile_ += steps;
    current_velocity_ = v_next;

    if (current_velocity_ > 0) {
        step_interval_us_ = 1000000 / current_velocity_;
    } else {
        step_interval_us_ = 0;
    }

    if (state_ == MotionState::EMERGENCY_STOPPING) {
        stopping_distance_steps_ = static_cast<uint32_t>(std::abs(current_position_ - static_cast<int32_t>(emergency_stop_start_pos_)));
        if (current_velocity_ == 0) {
            state_ = MotionState::COMPLETE;
        }
    } else if (current_position_ == target_position_) {
        state_ = MotionState::COMPLETE;
        current_velocity_ = 0;
        step_interval_us_ = 0;
    }

    return steps;
}

// BladePlanner implementation
BladePlanner::BladePlanner(uint32_t stroke_steps) noexcept
    : stroke_steps_(stroke_steps) {
    MotionLimits blade_limits{};
    blade_limits.max_velocity_steps_s = 5000;      // 50 mm/s
    blade_limits.max_accel_steps_s2 = 15000;       // 150 mm/s^2
    blade_limits.emergency_decel_steps_s2 = 40000; // 400 mm/s^2
    blade_limits.min_position_steps = 0;
    blade_limits.max_position_steps = stroke_steps;
    planner_.set_limits(blade_limits);
    planner_.set_current_position(0); // TDC = 0
}

void BladePlanner::start_cycle() noexcept {
    planner_.set_current_position(0);
    planner_.plan_move(stroke_steps_); // Move down to BDC
    phase_ = Phase::MOVING_DOWN;
    dwell_ms_ = 0;
}

void BladePlanner::emergency_stop() noexcept {
    planner_.emergency_stop();
    phase_ = Phase::EMERGENCY_STOPPED;
}

void BladePlanner::hard_stop() noexcept {
    planner_.hard_stop();
    phase_ = Phase::EMERGENCY_STOPPED;
}

bool BladePlanner::tick_1ms() noexcept {
    if (phase_ == Phase::AT_TDC || phase_ == Phase::EMERGENCY_STOPPED) {
        return false;
    }

    planner_.tick_1ms();

    if (phase_ == Phase::MOVING_DOWN) {
        if (!planner_.is_moving() && planner_.current_position_steps() >= static_cast<int32_t>(stroke_steps_)) {
            phase_ = Phase::AT_BDC;
            dwell_ms_ = 0;
        }
    } else if (phase_ == Phase::AT_BDC) {
        dwell_ms_++;
        if (dwell_ms_ >= 20) { // 20 ms dwell at bottom of stroke
            planner_.plan_move(0); // Return up to TDC
            phase_ = Phase::MOVING_UP;
        }
    } else if (phase_ == Phase::MOVING_UP) {
        if (!planner_.is_moving() && planner_.current_position_steps() <= 0) {
            phase_ = Phase::AT_TDC;
            return true; // Cycle complete!
        }
    }

    return false;
}

} // namespace papercutter::motion
