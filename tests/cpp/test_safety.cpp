#include <gtest/gtest.h>
#include "core/safety/safety.hpp"
#include "core/fsm/state_machine.hpp"
#include "core/motion/planner.hpp"

using namespace papercutter;
using namespace papercutter::safety;
using namespace papercutter::fsm;
using namespace papercutter::motion;
using namespace papercutter::protocol;

TEST(SafetyTest, WatchdogExpiryTripsHaltAt200Ms) {
    SafetySupervisor safety(200);
    StateMachine fsm;
    TrapezoidalPlanner back_gauge;
    BladePlanner blade;

    fsm.context().job_loaded = true;
    fsm.context().total_cuts = 1;
    fsm.handle_event(Event::START);
    EXPECT_EQ(fsm.state(), State::FEEDING);

    // Tick 199 ms -> not tripped
    for (int i = 0; i < 199; ++i) {
        safety.tick_1ms(fsm, back_gauge, blade, 0, true);
    }
    EXPECT_FALSE(safety.is_watchdog_tripped());
    EXPECT_EQ(fsm.state(), State::FEEDING);

    // 200th ms -> trips immediately
    safety.tick_1ms(fsm, back_gauge, blade, 0, true);
    EXPECT_TRUE(safety.is_watchdog_tripped());
    EXPECT_EQ(fsm.state(), State::ERROR_FAULT);
    EXPECT_EQ(fsm.context().latched_fault, FaultCode::COMM_TIMEOUT);
    EXPECT_TRUE(fsm.context().motors_halted);
}

TEST(SafetyTest, WatchdogPetPreventsTimeout) {
    SafetySupervisor safety(200);
    StateMachine fsm;
    TrapezoidalPlanner back_gauge;
    BladePlanner blade;

    fsm.context().job_loaded = true;
    fsm.context().total_cuts = 1;
    fsm.handle_event(Event::START);

    // Run 500 ms while petting watchdog every 100 ms
    for (int cycle = 0; cycle < 5; ++cycle) {
        for (int i = 0; i < 100; ++i) {
            safety.tick_1ms(fsm, back_gauge, blade, 0, true);
        }
        safety.pet_watchdog();
    }

    EXPECT_FALSE(safety.is_watchdog_tripped());
    EXPECT_EQ(fsm.state(), State::FEEDING);
}

TEST(SafetyTest, CrcStormDetection) {
    SafetySupervisor safety(200);
    StateMachine fsm;
    TrapezoidalPlanner back_gauge;
    BladePlanner blade;

    fsm.context().job_loaded = true;
    fsm.context().total_cuts = 1;
    fsm.handle_event(Event::START);

    // Simulate 6 CRC errors within 100 ms
    for (int i = 0; i < 100; ++i) {
        safety.tick_1ms(fsm, back_gauge, blade, 6, true);
    }

    EXPECT_EQ(fsm.state(), State::ERROR_FAULT);
    EXPECT_EQ(fsm.context().latched_fault, FaultCode::CRC_STORM);
    EXPECT_TRUE(fsm.context().motors_halted);
}

TEST(SafetyTest, MotionOverrunDetection) {
    SafetySupervisor safety(200);
    StateMachine fsm;
    TrapezoidalPlanner back_gauge;
    BladePlanner blade;

    fsm.context().job_loaded = true;
    fsm.context().total_cuts = 1;
    fsm.handle_event(Event::START);

    // Simulate mechanical boundary overrun
    back_gauge.set_current_position(105000); // Exceeds 100000 limit

    safety.tick_1ms(fsm, back_gauge, blade, 0, true);

    EXPECT_EQ(fsm.state(), State::ERROR_FAULT);
    EXPECT_EQ(fsm.context().latched_fault, FaultCode::MOTION_OVERRUN);
    EXPECT_TRUE(fsm.context().motors_halted);
}
