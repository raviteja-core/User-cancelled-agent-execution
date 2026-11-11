#pragma once

#include <cstddef>
#include <cstdint>

namespace papercutter::protocol {

// Computes CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no final XOR).
// Standard test: "123456789" -> 0x29B1.
uint16_t crc16_ccitt_false(const uint8_t* data, size_t length, uint16_t initial = 0xFFFF) noexcept;

} // namespace papercutter::protocol
