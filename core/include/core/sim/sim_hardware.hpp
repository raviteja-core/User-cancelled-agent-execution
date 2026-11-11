#pragma once

#include "core/fsm/state_machine.hpp"
#include "core/motion/planner.hpp"
#include "core/protocol/packet.hpp"
#include <cstddef>
#include <cstdint>

namespace papercutter::sim {

class SimulatedHardware {
public:
    explicit SimulatedHardware(uint32_t blade_stroke_steps = 5000) noexcept;

    // Advances hardware simulation by delta_ms ticks (typically 1 ms)
    void tick(uint32_t delta_ms, fsm::StateMachine& fsm, motion::TrapezoidalPlanner& back_gauge, motion::BladePlanner& blade) noexcept;

    // Actuator controls
    void set_clamp_engaged(bool engage) noexcept;

    // Fault injection hooks (for test suites)
    void inject_paper_jam(bool jam_active) noexcept;
    void inject_premature_blade_limit(bool trigger) noexcept;
    void inject_interlock_trip(bool trip) noexcept;

    // Sensor getters
    [[nodiscard]] bool feed_sensor() const noexcept { return feed_sensor_active_; }
    [[nodiscard]] bool clamp_switch() const noexcept { return clamp_switch_closed_; }
    [[nodiscard]] bool blade_tdc_switch() const noexcept { return blade_tdc_switch_; }
    [[nodiscard]] bool blade_bdc_switch() const noexcept { return blade_bdc_switch_; }
    [[nodiscard]] uint8_t limit_switch_bitmask() const noexcept;
    [[nodiscard]] bool paper_jam_active() const noexcept { return paper_jam_injected_; }

    // Telemetry generation helper
    protocol::TelemetryPayload generate_telemetry(
        const fsm::StateMachine& fsm,
        const motion::TrapezoidalPlanner& back_gauge,
        const motion::BladePlanner& blade,
        uint16_t crc_errors,
        uint16_t dropped_packets,
        uint16_t timeout_errors
    ) const noexcept;

private:
    uint32_t blade_stroke_steps_{5000};

    // Clamp model
    bool clamp_commanded_{false};
    uint32_t clamp_timer_ms_{0};
    static constexpr uint32_t CLAMP_ENGAGE_DELAY_MS = 150; // 150 ms clamp travel
    bool clamp_switch_closed_{false};

    // Sensor states
    bool feed_sensor_active_{false};
    bool blade_tdc_switch_{true};
    bool blade_bdc_switch_{false};

    // Fault injection flags
    bool paper_jam_injected_{false};
    bool premature_limit_injected_{false};
    bool interlock_tripped_{false};
};

} // namespace papercutter::sim
