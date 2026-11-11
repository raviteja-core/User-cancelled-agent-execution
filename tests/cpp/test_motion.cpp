#include <gtest/gtest.h>
#include "core/motion/planner.hpp"

using namespace papercutter::motion;

TEST(MotionPlannerTest, LongMoveTrapezoidalProfile) {
    MotionLimits limits{};
    limits.max_velocity_steps_s = 10000; // 10000 steps/s
    limits.max_accel_steps_s2 = 20000;   // 20000 steps/s^2

    TrapezoidalPlanner planner(limits);
    planner.set_current_position(0);

    const int32_t target = 25000; // 250 mm
    ASSERT_TRUE(planner.plan_move(target));

    uint32_t total_steps = 0;
    uint32_t prev_velocity = 0;

    while (planner.is_moving()) {
        uint32_t steps = planner.tick_1ms();
        total_steps += steps;

        // Verify peak velocity within limit
        EXPECT_LE(planner.current_velocity_steps_s(), limits.max_velocity_steps_s);

        // Verify acceleration limit: dv <= a / 1000 + 1 (allow rounding)
        uint32_t dv = (planner.current_velocity_steps_s() > prev_velocity)
            ? (planner.current_velocity_steps_s() - prev_velocity)
            : (prev_velocity - planner.current_velocity_steps_s());
        EXPECT_LE(dv, (limits.max_accel_steps_s2 / 1000) + 1);

        // Verify no negative intervals
        if (planner.current_velocity_steps_s() > 0) {
            EXPECT_GT(planner.step_interval_us(), 0);
        }

        prev_velocity = planner.current_velocity_steps_s();
    }

    // Verify exact final position and total steps
    EXPECT_EQ(planner.current_position_steps(), target);
    EXPECT_EQ(total_steps, target);
    EXPECT_EQ(planner.current_velocity_steps_s(), 0);
    EXPECT_EQ(planner.state(), MotionState::COMPLETE);
}

TEST(MotionPlannerTest, ShortMoveTriangularProfile) {
    MotionLimits limits{};
    limits.max_velocity_steps_s = 10000;
    limits.max_accel_steps_s2 = 20000;

    TrapezoidalPlanner planner(limits);
    planner.set_current_position(0);

    const int32_t target = 500; // Short move: 5 mm
    ASSERT_TRUE(planner.plan_move(target));

    uint32_t total_steps = 0;
    while (planner.is_moving()) {
        uint32_t steps = planner.tick_1ms();
        total_steps += steps;
        EXPECT_LE(planner.current_velocity_steps_s(), limits.max_velocity_steps_s);
    }

    EXPECT_EQ(planner.current_position_steps(), target);
    EXPECT_EQ(total_steps, target);
}

TEST(MotionPlannerTest, ReverseMove) {
    MotionLimits limits{};
    TrapezoidalPlanner planner(limits);
    planner.set_current_position(20000);

    const int32_t target = 5000;
    ASSERT_TRUE(planner.plan_move(target));

    uint32_t total_steps = 0;
    while (planner.is_moving()) {
        total_steps += planner.tick_1ms();
    }

    EXPECT_EQ(planner.current_position_steps(), target);
    EXPECT_EQ(total_steps, 15000);
}

TEST(MotionPlannerTest, EmergencyStopAppliesDecelerationAndRecordsDistance) {
    MotionLimits limits{};
    limits.max_velocity_steps_s = 10000;
    limits.max_accel_steps_s2 = 20000;
    limits.emergency_decel_steps_s2 = 50000; // 500 mm/s^2

    TrapezoidalPlanner planner(limits);
    planner.set_current_position(0);
    planner.plan_move(30000);

    // Run for 300 ms to build up velocity
    for (int i = 0; i < 300; ++i) {
        planner.tick_1ms();
    }
    ASSERT_GT(planner.current_velocity_steps_s(), 1000);
    int32_t estop_triggered_pos = planner.current_position_steps();

    // Trigger emergency stop
    planner.emergency_stop();
    EXPECT_EQ(planner.state(), MotionState::EMERGENCY_STOPPING);

    // Tick until completely halted
    while (planner.is_moving()) {
        planner.tick_1ms();
    }

    EXPECT_EQ(planner.current_velocity_steps_s(), 0);
    EXPECT_GT(planner.stopping_distance_steps(), 0);
    EXPECT_EQ(planner.stopping_distance_steps(), planner.current_position_steps() - estop_triggered_pos);
}

TEST(MotionPlannerTest, BladePlannerFullCycle) {
    BladePlanner blade(4000); // 40 mm stroke
    EXPECT_TRUE(blade.is_at_tdc());
    EXPECT_EQ(blade.position_steps(), 0);

    blade.start_cycle();
    EXPECT_EQ(blade.phase(), BladePlanner::Phase::MOVING_DOWN);

    bool finished = false;
    int ticks = 0;
    while (!finished && ticks < 5000) {
        finished = blade.tick_1ms();
        ticks++;
    }

    EXPECT_TRUE(finished);
    EXPECT_TRUE(blade.is_at_tdc());
    EXPECT_EQ(blade.position_steps(), 0);
}
