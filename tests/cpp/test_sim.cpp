#include <gtest/gtest.h>
#include "core/sim/sim_hardware.hpp"
#include "core/fsm/state_machine.hpp"
#include "core/motion/planner.hpp"

using namespace papercutter;
using namespace papercutter::sim;
using namespace papercutter::fsm;
using namespace papercutter::motion;
using namespace papercutter::protocol;

TEST(SimHardwareTest, InitialSensorStates) {
    SimulatedHardware sim(5000);
    EXPECT_TRUE(sim.blade_tdc_switch());
    EXPECT_FALSE(sim.blade_bdc_switch());
    EXPECT_FALSE(sim.clamp_switch());
    EXPECT_FALSE(sim.feed_sensor());

    uint8_t mask = sim.limit_switch_bitmask();
    EXPECT_TRUE(mask & static_cast<uint8_t>(LimitSwitchBit::BLADE_TDC));
    EXPECT_TRUE(mask & static_cast<uint8_t>(LimitSwitchBit::SAFETY_GUARD));
    EXPECT_FALSE(mask & static_cast<uint8_t>(LimitSwitchBit::BLADE_BDC));
    EXPECT_FALSE(mask & static_cast<uint8_t>(LimitSwitchBit::CLAMP_SWITCH));
}

TEST(SimHardwareTest, ClampEngageDelayModel) {
    SimulatedHardware sim;
    StateMachine fsm;
    TrapezoidalPlanner back_gauge;
    BladePlanner blade;

    fsm.context().job_loaded = true;
    fsm.context().total_cuts = 1;
    fsm.handle_event(Event::START);
    EXPECT_EQ(fsm.state(), State::FEEDING);

    // Fast-forward back gauge until position reached
    while (fsm.state() == State::FEEDING) {
        sim.tick(1, fsm, back_gauge, blade);
    }
    EXPECT_EQ(fsm.state(), State::CLAMPING);
    EXPECT_FALSE(sim.clamp_switch());

    // Advance 140 ms -> clamp still not closed (< 150 ms)
    for (int i = 0; i < 140; ++i) {
        sim.tick(1, fsm, back_gauge, blade);
    }
    EXPECT_EQ(fsm.state(), State::CLAMPING);
    EXPECT_FALSE(sim.clamp_switch());

    // Advance another 15 ms -> clamp closes and triggers transition to CUTTING
    for (int i = 0; i < 15; ++i) {
        sim.tick(1, fsm, back_gauge, blade);
    }
    EXPECT_EQ(fsm.state(), State::CUTTING);
    EXPECT_TRUE(sim.clamp_switch());
}

TEST(SimHardwareTest, PaperJamFaultInjection) {
    SimulatedHardware sim;
    StateMachine fsm;
    TrapezoidalPlanner back_gauge;
    BladePlanner blade;

    fsm.context().job_loaded = true;
    fsm.context().total_cuts = 1;
    fsm.handle_event(Event::START);
    EXPECT_EQ(fsm.state(), State::FEEDING);

    // Inject paper jam
    sim.inject_paper_jam(true);
    EXPECT_TRUE(sim.paper_jam_active());

    // Run ticks until 3.1 s have elapsed
    for (int i = 0; i < 3100; ++i) {
        sim.tick(1, fsm, back_gauge, blade);
    }

    EXPECT_EQ(fsm.state(), State::ERROR_JAM);
    EXPECT_EQ(fsm.context().latched_fault, FaultCode::PAPER_JAM);
    EXPECT_TRUE(fsm.context().motors_halted);
}

TEST(SimHardwareTest, PrematureBladeLimitInjectionHalts) {
    SimulatedHardware sim;
    StateMachine fsm;
    TrapezoidalPlanner back_gauge;
    BladePlanner blade;

    fsm.context().job_loaded = true;
    fsm.context().total_cuts = 1;
    fsm.handle_event(Event::START);
    EXPECT_EQ(fsm.state(), State::FEEDING);

    // Inject premature blade limit while feeding
    sim.inject_premature_blade_limit(true);
    sim.tick(1, fsm, back_gauge, blade);

    EXPECT_EQ(fsm.state(), State::ERROR_FAULT);
    EXPECT_EQ(fsm.context().latched_fault, FaultCode::PREMATURE_LIMIT);
    EXPECT_TRUE(fsm.context().motors_halted);
}

TEST(SimHardwareTest, TelemetryPayloadGeneration) {
    SimulatedHardware sim;
    StateMachine fsm;
    TrapezoidalPlanner back_gauge;
    BladePlanner blade;

    back_gauge.set_current_position(12500); // 125.00 mm
    auto telem = sim.generate_telemetry(fsm, back_gauge, blade, 2, 1, 0);

    EXPECT_EQ(telem.state, static_cast<uint8_t>(State::IDLE));
    EXPECT_EQ(telem.fault_code, static_cast<uint16_t>(FaultCode::NONE));
    EXPECT_EQ(telem.back_gauge_pos, 12500);
    EXPECT_EQ(telem.blade_pos, 0);
    EXPECT_EQ(telem.crc_errors, 2);
    EXPECT_EQ(telem.dropped_packets, 1);
    EXPECT_EQ(telem.timeout_errors, 0);
}
