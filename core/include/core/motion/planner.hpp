#pragma once

#include <cstddef>
#include <cstdint>

namespace papercutter::motion {

struct MotionLimits {
    uint32_t steps_per_mm{100};              // 100 steps/mm -> 1 step = 0.01 mm = 10 um
    uint32_t max_velocity_steps_s{10000};    // 100 mm/s
    uint32_t max_accel_steps_s2{20000};      // 200 mm/s^2
    uint32_t emergency_decel_steps_s2{50000};// 500 mm/s^2
    int32_t min_position_steps{0};
    int32_t max_position_steps{100000};      // 1000 mm max travel
};

enum class MotionState : uint8_t {
    IDLE = 0,
    ACCELERATING = 1,
    CRUISING = 2,
    DECELERATING = 3,
    EMERGENCY_STOPPING = 4,
    COMPLETE = 5
};

class TrapezoidalPlanner {
public:
    explicit TrapezoidalPlanner(const MotionLimits& limits = MotionLimits{}) noexcept;

    // Plans a move from current position to target position (in steps).
    // Returns false if target is outside allowable limits or already at target.
    bool plan_move(int32_t target_position_steps) noexcept;

    // Triggers an emergency stop, applying emergency deceleration limit.
    void emergency_stop() noexcept;

    // Hard immediate stop (clamps velocity to 0)
    void hard_stop() noexcept;

    // Advances planner by 1 ms tick.
    // Returns number of steps executed in this tick (0 or more).
    uint32_t tick_1ms() noexcept;

    // Setters
    void set_current_position(int32_t pos_steps) noexcept;
    void set_limits(const MotionLimits& limits) noexcept;

    // Getters
    [[nodiscard]] int32_t current_position_steps() const noexcept { return current_position_; }
    [[nodiscard]] int32_t target_position_steps() const noexcept { return target_position_; }
    [[nodiscard]] uint32_t current_velocity_steps_s() const noexcept { return current_velocity_; }
    [[nodiscard]] uint32_t peak_velocity_steps_s() const noexcept { return peak_velocity_; }
    [[nodiscard]] uint32_t step_interval_us() const noexcept { return step_interval_us_; }
    [[nodiscard]] MotionState state() const noexcept { return state_; }
    [[nodiscard]] bool is_moving() const noexcept { return state_ != MotionState::IDLE && state_ != MotionState::COMPLETE; }
    [[nodiscard]] uint32_t stopping_distance_steps() const noexcept { return stopping_distance_steps_; }
    [[nodiscard]] int direction() const noexcept { return direction_; } // +1 forward, -1 backward, 0 stationary

private:
    MotionLimits limits_{};
    MotionState state_{MotionState::IDLE};

    int32_t current_position_{0};
    int32_t target_position_{0};
    int direction_{0};

    uint32_t current_velocity_{0};       // steps / second
    uint32_t cruise_velocity_{0};        // target cruise velocity
    uint32_t peak_velocity_{0};          // tracked for verification

    uint32_t total_distance_steps_{0};
    uint32_t steps_moved_in_profile_{0};
    uint32_t accel_steps_{0};
    uint32_t decel_steps_{0};

    uint32_t step_interval_us_{0};
    uint32_t stopping_distance_steps_{0};
    uint32_t emergency_stop_start_pos_{0};

    // Sub-step fractional accumulator for 1 ms tick integration
    uint32_t step_remainder_{0};
};

// Blade motion profile: downstroke from TDC (0) to BDC (stroke_steps), then return to TDC
class BladePlanner {
public:
    enum class Phase : uint8_t {
        AT_TDC = 0,
        MOVING_DOWN = 1,
        AT_BDC = 2,
        MOVING_UP = 3,
        EMERGENCY_STOPPED = 4
    };

    explicit BladePlanner(uint32_t stroke_steps = 5000) noexcept; // 50 mm stroke default

    void start_cycle() noexcept;
    void emergency_stop() noexcept;
    void hard_stop() noexcept;

    // Advances blade by 1 ms tick.
    // Returns true if complete cycle (down and back up to TDC) finished.
    bool tick_1ms() noexcept;

    [[nodiscard]] Phase phase() const noexcept { return phase_; }
    [[nodiscard]] int32_t position_steps() const noexcept { return planner_.current_position_steps(); }
    [[nodiscard]] bool is_at_tdc() const noexcept { return phase_ == Phase::AT_TDC; }
    [[nodiscard]] bool is_at_bdc() const noexcept { return phase_ == Phase::AT_BDC; }
    [[nodiscard]] uint32_t stopping_distance_steps() const noexcept { return planner_.stopping_distance_steps(); }
    [[nodiscard]] const TrapezoidalPlanner& underlying_planner() const noexcept { return planner_; }

private:
    uint32_t stroke_steps_{5000};
    Phase phase_{Phase::AT_TDC};
    TrapezoidalPlanner planner_;
    uint32_t dwell_ms_{0};
};

} // namespace papercutter::motion
