#include <gtest/gtest.h>
#include "core/protocol/crc16.hpp"
#include <cstring>

TEST(Crc16Test, StandardCheckVector) {
    const char* input = "123456789";
    uint16_t crc = papercutter::protocol::crc16_ccitt_false(
        reinterpret_cast<const uint8_t*>(input),
        std::strlen(input)
    );
    EXPECT_EQ(crc, 0x29B1);
}

TEST(Crc16Test, EmptyBufferReturnsInitial) {
    uint16_t crc = papercutter::protocol::crc16_ccitt_false(nullptr, 0, 0xFFFF);
    EXPECT_EQ(crc, 0xFFFF);
}

TEST(Crc16Test, SingleByte) {
    uint8_t data[] = { 0x01 };
    uint16_t crc = papercutter::protocol::crc16_ccitt_false(data, 1);
    // 0xFFFF ^ (0x01 << 8) = 0xFEFF. After 8 rounds:
    EXPECT_NE(crc, 0xFFFF);
}
