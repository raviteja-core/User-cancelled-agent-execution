#include "core/protocol/crc16.hpp"
#include <array>

namespace papercutter::protocol {

namespace {

constexpr auto generate_crc_table() noexcept {
    std::array<uint16_t, 256> table{};
    for (size_t i = 0; i < 256; ++i) {
        uint16_t curr = static_cast<uint16_t>(i << 8);
        for (int b = 0; b < 8; ++b) {
            if (curr & 0x8000) {
                curr = static_cast<uint16_t>((curr << 1) ^ 0x1021);
            } else {
                curr = static_cast<uint16_t>(curr << 1);
            }
        }
        table[i] = curr;
    }
    return table;
}

constexpr auto CRC16_TABLE = generate_crc_table();

} // namespace

uint16_t crc16_ccitt_false(const uint8_t* data, size_t length, uint16_t initial) noexcept {
    if (data == nullptr || length == 0) {
        return initial;
    }
    uint16_t crc = initial;
    for (size_t i = 0; i < length; ++i) {
        uint8_t byte = data[i];
        crc = static_cast<uint16_t>((crc << 8) ^ CRC16_TABLE[(crc >> 8) ^ byte]);
    }
    return crc;
}

} // namespace papercutter::protocol
