#pragma once

#include <cstddef>
#include <cstdint>
#include <array>

namespace papercutter::protocol {

constexpr uint8_t SOF1 = 0xAA;
constexpr uint8_t SOF2 = 0x55;
constexpr size_t MIN_PAYLOAD_LEN = 2;   // SEQ (1) + CMD (1)
constexpr size_t MAX_PAYLOAD_LEN = 250; // SEQ (1) + CMD (1) + PAYLOAD (max 248)
constexpr size_t MAX_PAYLOAD_DATA_LEN = 248;
constexpr size_t PREAMBLE_SIZE = 5;      // SOF1, SOF2, LEN, SEQ, CMD
constexpr size_t CRC_SIZE = 2;          // CRC high, CRC low
constexpr size_t MAX_PACKET_SIZE = PREAMBLE_SIZE + MAX_PAYLOAD_DATA_LEN + CRC_SIZE; // 255
constexpr uint32_t INTER_BYTE_TIMEOUT_MS = 50;

enum class Command : uint8_t {
    HELLO = 0x01,
    HEARTBEAT = 0x02,
    LOAD_JOB = 0x10,
    START = 0x11,
    PAUSE_RESUME = 0x12,
    ABORT = 0x13,
    ESTOP = 0x20,
    RESET_FAULT = 0x21,
    TELEMETRY = 0x30,
    ACK = 0x7E,
    NACK = 0x7F,
    SIM_INJECT = 0x90
};

enum class FaultCode : uint16_t {
    NONE = 0x0000,
    PAPER_JAM = 0x0001,
    COMM_TIMEOUT = 0x0002,
    PREMATURE_LIMIT = 0x0003,
    ESTOP = 0x0004,
    CRC_STORM = 0x0005,
    MOTION_OVERRUN = 0x0006,
    INVALID_JOB = 0x0007
};

enum class State : uint8_t {
    IDLE = 0,
    FEEDING = 1,
    CLAMPING = 2,
    CUTTING = 3,
    ERROR_JAM = 4,
    ERROR_FAULT = 5
};

enum class LimitSwitchBit : uint8_t {
    NONE = 0x00,
    BLADE_TDC = 0x01,
    BLADE_BDC = 0x02,
    CLAMP_SWITCH = 0x04,
    SAFETY_GUARD = 0x08
};

struct Packet {
    uint8_t seq{0};
    uint8_t cmd{0};
    uint8_t payload_len{0};
    std::array<uint8_t, MAX_PAYLOAD_DATA_LEN> payload{};

    constexpr bool operator==(const Packet& other) const noexcept {
        if (seq != other.seq || cmd != other.cmd || payload_len != other.payload_len) {
            return false;
        }
        for (size_t i = 0; i < payload_len; ++i) {
            if (payload[i] != other.payload[i]) {
                return false;
            }
        }
        return true;
    }
};

struct TelemetryPayload {
    uint8_t state{0};
    uint16_t fault_code{0};
    int32_t back_gauge_pos{0};
    int32_t blade_pos{0};
    uint8_t feed_sensor{0};
    uint8_t limit_switches{0};
    uint8_t cut_index{0};
    uint16_t crc_errors{0};
    uint16_t dropped_packets{0};
    uint16_t timeout_errors{0};
    uint8_t clamp_engaged{0};
    uint16_t reserved{0};
};

constexpr size_t TELEMETRY_PAYLOAD_SIZE = 23;

} // namespace papercutter::protocol
