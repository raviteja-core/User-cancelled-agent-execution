#pragma once

#include "core/protocol/packet.hpp"
#include <cstddef>
#include <cstdint>
#include <array>

namespace papercutter::protocol {

enum class ParserState : uint8_t {
    WAIT_SOF1 = 0,
    WAIT_SOF2 = 1,
    READ_LEN = 2,
    READ_BODY = 3,
    READ_CRC1 = 4,
    READ_CRC2 = 5
};

class PacketParser {
public:
    constexpr PacketParser() noexcept = default;

    // Resets parser state to WAIT_SOF1
    void reset() noexcept;

    // Process a single byte. Returns true if a full valid packet has been assembled.
    bool process_byte(uint8_t byte, Packet& out_pkt) noexcept;

    // Advance parser time by delta_ms. Checks 50 ms inter-byte timeout.
    // Returns true if an inter-byte timeout caused a parser reset.
    bool tick(uint32_t delta_ms) noexcept;

    [[nodiscard]] constexpr ParserState state() const noexcept { return state_; }
    [[nodiscard]] constexpr uint16_t crc_errors() const noexcept { return crc_errors_; }
    [[nodiscard]] constexpr uint16_t dropped_packets() const noexcept { return dropped_packets_; }
    [[nodiscard]] constexpr uint16_t timeout_errors() const noexcept { return timeout_errors_; }

private:
    ParserState state_{ParserState::WAIT_SOF1};
    uint8_t expected_len_{0};
    uint8_t body_bytes_read_{0};
    uint8_t crc_high_{0};
    uint8_t crc_low_{0};
    uint32_t time_since_last_byte_ms_{0};

    // Internal buffer: LEN byte + SEQ + CMD + PAYLOAD
    // Max LEN is 250, so buffer size 251 is sufficient
    std::array<uint8_t, MAX_PAYLOAD_LEN + 1> body_buf_{};

    uint16_t crc_errors_{0};
    uint16_t dropped_packets_{0};
    uint16_t timeout_errors_{0};
};

} // namespace papercutter::protocol
