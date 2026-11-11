#pragma once

#include "core/protocol/packet.hpp"
#include <cstddef>
#include <cstdint>

namespace papercutter::protocol {

// Serializes a packet into big-endian wire format:
// [0xAA, 0x55, LEN, SEQ, CMD, PAYLOAD..., CRC_HIGH, CRC_LOW]
// Returns true on success, false if buffer is too small or packet is invalid.
bool encode_packet(const Packet& pkt, uint8_t* out_buf, size_t out_capacity, size_t& out_size) noexcept;

// Helper to encode a TelemetryPayload directly into a packet wire buffer
bool encode_telemetry(const TelemetryPayload& telem, uint8_t seq, uint8_t* out_buf, size_t out_capacity, size_t& out_size) noexcept;

// Helper to encode an ACK packet
bool encode_ack(uint8_t seq, uint8_t acked_seq, uint8_t status, uint8_t* out_buf, size_t out_capacity, size_t& out_size) noexcept;

// Helper to encode a NACK packet
bool encode_nack(uint8_t seq, uint8_t rejected_seq, uint8_t reason, uint8_t* out_buf, size_t out_capacity, size_t& out_size) noexcept;

} // namespace papercutter::protocol
