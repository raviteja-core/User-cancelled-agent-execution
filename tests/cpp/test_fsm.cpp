#include <gtest/gtest.h>
#include "core/fsm/state_machine.hpp"

using namespace papercutter;
using namespace papercutter::fsm;
using namespace papercutter::protocol;

TEST(FsmTest, InitialStateIsIdleAndHalted) {
    StateMachine fsm;
    EXPECT_EQ(fsm.state(), State::IDLE);
    EXPECT_TRUE(fsm.context().motors_halted);
    EXPECT_EQ(fsm.context().latched_fault, FaultCode::NONE);
}

TEST(FsmTest, StartGuardRejectsWithoutJob) {
    StateMachine fsm;
    auto res = fsm.handle_event(Event::START);
    EXPECT_EQ(res, TransitionResult::GUARD_FAILED);
    EXPECT_EQ(fsm.state(), State::IDLE);
    EXPECT_TRUE(fsm.context().motors_halted);
}

TEST(FsmTest, NormalJobCycleExecution) {
    StateMachine fsm;

    // Load a 2-cut job
    fsm.context().job_loaded = true;
    fsm.context().job_id = 101;
    fsm.context().total_cuts = 2;
    fsm.context().current_cut = 0;
    fsm.context().interlocks_safe = true;
    fsm.context().estop_asserted = false;

    // 1. START
    ASSERT_EQ(fsm.handle_event(Event::START), TransitionResult::SUCCESS);
    EXPECT_EQ(fsm.state(), State::FEEDING);
    EXPECT_TRUE(fsm.context().back_gauge_moving);
    EXPECT_FALSE(fsm.context().motors_halted);

    // 2. Feed position reached with paper
    fsm.context().feed_sensor_active = true;
    ASSERT_EQ(fsm.handle_event(Event::FEED_POS_REACHED), TransitionResult::SUCCESS);
    EXPECT_EQ(fsm.state(), State::CLAMPING);
    EXPECT_FALSE(fsm.context().back_gauge_moving);
    EXPECT_TRUE(fsm.context().clamp_actuating);

    // 3. Clamp confirmed
    fsm.context().clamp_closed = true;
    ASSERT_EQ(fsm.handle_event(Event::CLAMP_CONFIRMED), TransitionResult::SUCCESS);
    EXPECT_EQ(fsm.state(), State::CUTTING);
    EXPECT_FALSE(fsm.context().clamp_actuating);
    EXPECT_TRUE(fsm.context().blade_moving);

    // 4. Cut 1 complete -> next cut (cut index becomes 1)
    ASSERT_EQ(fsm.handle_event(Event::CUT_COMPLETE), TransitionResult::SUCCESS);
    EXPECT_EQ(fsm.state(), State::FEEDING);
    EXPECT_EQ(fsm.context().current_cut, 1);
    EXPECT_TRUE(fsm.context().back_gauge_moving);

    // 5. Feed position reached for cut 2
    ASSERT_EQ(fsm.handle_event(Event::FEED_POS_REACHED), TransitionResult::SUCCESS);
    EXPECT_EQ(fsm.state(), State::CLAMPING);

    // 6. Clamp confirmed
    ASSERT_EQ(fsm.handle_event(Event::CLAMP_CONFIRMED), TransitionResult::SUCCESS);
    EXPECT_EQ(fsm.state(), State::CUTTING);

    // 7. Cut 2 complete -> job finished -> IDLE
    ASSERT_EQ(fsm.handle_event(Event::CUT_COMPLETE), TransitionResult::SUCCESS);
    EXPECT_EQ(fsm.state(), State::IDLE);
    EXPECT_TRUE(fsm.context().motors_halted);
    EXPECT_FALSE(fsm.context().job_loaded);
}

TEST(FsmTest, FeedingJamTimeoutLatchesFault) {
    StateMachine fsm;
    fsm.context().job_loaded = true;
    fsm.context().total_cuts = 1;
    fsm.handle_event(Event::START);
    EXPECT_EQ(fsm.state(), State::FEEDING);

    // Tick 2900 ms -> no timeout yet
    fsm.tick(2900);
    EXPECT_EQ(fsm.state(), State::FEEDING);

    // Tick 200 ms -> exceeds 3000 ms -> transitions to ERROR_JAM
    fsm.tick(200);
    EXPECT_EQ(fsm.state(), State::ERROR_JAM);
    EXPECT_TRUE(fsm.context().motors_halted);
    EXPECT_EQ(fsm.context().latched_fault, FaultCode::PAPER_JAM);

    // Cannot start again while latched
    EXPECT_EQ(fsm.handle_event(Event::START), TransitionResult::ILLEGAL_TRANSITION);
    EXPECT_EQ(fsm.state(), State::ERROR_JAM);
}

TEST(FsmTest, EstopImmediateHaltAndLatch) {
    StateMachine fsm;
    fsm.context().job_loaded = true;
    fsm.context().total_cuts = 1;
    fsm.handle_event(Event::START);
    EXPECT_EQ(fsm.state(), State::FEEDING);

    // Trigger ESTOP
    fsm.force_estop();
    EXPECT_EQ(fsm.state(), State::ERROR_FAULT);
    EXPECT_TRUE(fsm.context().motors_halted);
    EXPECT_EQ(fsm.context().latched_fault, FaultCode::ESTOP);

    // Reset while E-stop active must fail
    auto res = fsm.handle_event(Event::RESET_FAULT);
    EXPECT_EQ(res, TransitionResult::GUARD_FAILED);
    EXPECT_EQ(fsm.state(), State::ERROR_FAULT);

    // Clear physical E-stop and reset
    fsm.context().estop_asserted = false;
    res = fsm.handle_event(Event::RESET_FAULT);
    EXPECT_EQ(res, TransitionResult::SUCCESS);
    EXPECT_EQ(fsm.state(), State::IDLE);
    EXPECT_EQ(fsm.context().latched_fault, FaultCode::NONE);
}

TEST(FsmTest, HeartbeatTimeoutHalt) {
    StateMachine fsm;
    fsm.context().job_loaded = true;
    fsm.context().total_cuts = 1;
    fsm.handle_event(Event::START);

    ASSERT_EQ(fsm.handle_event(Event::HEARTBEAT_TIMEOUT), TransitionResult::SUCCESS);
    EXPECT_EQ(fsm.state(), State::ERROR_FAULT);
    EXPECT_TRUE(fsm.context().motors_halted);
    EXPECT_EQ(fsm.context().latched_fault, FaultCode::COMM_TIMEOUT);
}

TEST(FsmTest, PrematureLimitSwitchHalt) {
    StateMachine fsm;
    // When in IDLE, unexpected blade limit trips
    ASSERT_EQ(fsm.handle_event(Event::PREMATURE_LIMIT), TransitionResult::SUCCESS);
    EXPECT_EQ(fsm.state(), State::ERROR_FAULT);
    EXPECT_TRUE(fsm.context().motors_halted);
    EXPECT_EQ(fsm.context().latched_fault, FaultCode::PREMATURE_LIMIT);
}

TEST(FsmTest, IllegalTransitionsRejected) {
    StateMachine fsm; // In IDLE
    // FEED_POS_REACHED is illegal from IDLE
    EXPECT_EQ(fsm.handle_event(Event::FEED_POS_REACHED), TransitionResult::ILLEGAL_TRANSITION);
    // CLAMP_CONFIRMED is illegal from IDLE
    EXPECT_EQ(fsm.handle_event(Event::CLAMP_CONFIRMED), TransitionResult::ILLEGAL_TRANSITION);
    // CUT_COMPLETE is illegal from IDLE
    EXPECT_EQ(fsm.handle_event(Event::CUT_COMPLETE), TransitionResult::ILLEGAL_TRANSITION);
    // RESET_FAULT is illegal from IDLE
    EXPECT_EQ(fsm.handle_event(Event::RESET_FAULT), TransitionResult::ILLEGAL_TRANSITION);
    EXPECT_EQ(fsm.state(), State::IDLE);
}
