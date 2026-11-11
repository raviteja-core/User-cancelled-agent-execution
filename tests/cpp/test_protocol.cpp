#include <gtest/gtest.h>
#include "core/protocol/crc16.hpp"
#include "core/protocol/packet.hpp"
#include "core/protocol/encoder.hpp"
#include "core/protocol/parser.hpp"
#include <vector>
#include <random>

using namespace papercutter::protocol;

TEST(ProtocolTest, EncodeAndParseHello) {
    Packet pkt;
    pkt.seq = 1;
    pkt.cmd = static_cast<uint8_t>(Command::HELLO);
    pkt.payload_len = 3;
    pkt.payload[0] = 1;   // proto
    pkt.payload[1] = 0;   // fw high
    pkt.payload[2] = 100; // fw low

    std::array<uint8_t, MAX_PACKET_SIZE> wire{};
    size_t wire_size = 0;
    bool ok = encode_packet(pkt, wire.data(), wire.size(), wire_size);
    ASSERT_TRUE(ok);
    EXPECT_EQ(wire_size, 10); // 5 (hdr) + 3 (payload) + 2 (crc)

    // Expected wire: aa 55 05 01 01 01 00 64 8a e6
    EXPECT_EQ(wire[0], 0xAA);
    EXPECT_EQ(wire[1], 0x55);
    EXPECT_EQ(wire[2], 0x05);
    EXPECT_EQ(wire[3], 0x01);
    EXPECT_EQ(wire[4], 0x01);
    EXPECT_EQ(wire[5], 0x01);
    EXPECT_EQ(wire[6], 0x00);
    EXPECT_EQ(wire[7], 0x64);
    EXPECT_EQ(wire[8], 0x8A);
    EXPECT_EQ(wire[9], 0xE6);

    PacketParser parser;
    Packet decoded;
    bool parsed = false;
    for (size_t i = 0; i < wire_size; ++i) {
        parsed = parser.process_byte(wire[i], decoded);
    }
    ASSERT_TRUE(parsed);
    EXPECT_EQ(decoded.seq, 1);
    EXPECT_EQ(decoded.cmd, static_cast<uint8_t>(Command::HELLO));
    EXPECT_EQ(decoded.payload_len, 3);
    EXPECT_EQ(decoded.payload[0], 1);
    EXPECT_EQ(decoded.payload[1], 0);
    EXPECT_EQ(decoded.payload[2], 100);
}

TEST(ProtocolTest, EncodeAndParseHeartbeat) {
    Packet pkt;
    pkt.seq = 2;
    pkt.cmd = static_cast<uint8_t>(Command::HEARTBEAT);
    pkt.payload_len = 0;

    std::array<uint8_t, MAX_PACKET_SIZE> wire{};
    size_t wire_size = 0;
    ASSERT_TRUE(encode_packet(pkt, wire.data(), wire.size(), wire_size));
    EXPECT_EQ(wire_size, 7);

    // Expected wire: aa 55 02 02 02 e4 dc
    EXPECT_EQ(wire[0], 0xAA);
    EXPECT_EQ(wire[1], 0x55);
    EXPECT_EQ(wire[2], 0x02);
    EXPECT_EQ(wire[3], 0x02);
    EXPECT_EQ(wire[4], 0x02);
    EXPECT_EQ(wire[5], 0xE4);
    EXPECT_EQ(wire[6], 0xDC);

    PacketParser parser;
    Packet decoded;
    bool parsed = false;
    for (size_t i = 0; i < wire_size; ++i) {
        parsed = parser.process_byte(wire[i], decoded);
    }
    ASSERT_TRUE(parsed);
    EXPECT_EQ(decoded.seq, 2);
    EXPECT_EQ(decoded.cmd, static_cast<uint8_t>(Command::HEARTBEAT));
    EXPECT_EQ(decoded.payload_len, 0);
}

TEST(ProtocolTest, EncodeAndParseTelemetry) {
    TelemetryPayload telem{};
    telem.state = 1;
    telem.fault_code = 0;
    telem.back_gauge_pos = 15000;
    telem.blade_pos = 0;
    telem.feed_sensor = 1;
    telem.limit_switches = 0x01;
    telem.cut_index = 1;

    std::array<uint8_t, MAX_PACKET_SIZE> wire{};
    size_t wire_size = 0;
    ASSERT_TRUE(encode_telemetry(telem, 9, wire.data(), wire.size(), wire_size));
    EXPECT_EQ(wire_size, 5 + TELEMETRY_PAYLOAD_SIZE + 2);

    PacketParser parser;
    Packet decoded;
    bool parsed = false;
    for (size_t i = 0; i < wire_size; ++i) {
        parsed = parser.process_byte(wire[i], decoded);
    }
    ASSERT_TRUE(parsed);
    EXPECT_EQ(decoded.seq, 9);
    EXPECT_EQ(decoded.cmd, static_cast<uint8_t>(Command::TELEMETRY));
    EXPECT_EQ(decoded.payload_len, TELEMETRY_PAYLOAD_SIZE);
}

TEST(ProtocolTest, CorruptedCrcIncrementCounterAndDrops) {
    Packet pkt;
    pkt.seq = 1;
    pkt.cmd = static_cast<uint8_t>(Command::HEARTBEAT);
    pkt.payload_len = 0;

    std::array<uint8_t, MAX_PACKET_SIZE> wire{};
    size_t wire_size = 0;
    ASSERT_TRUE(encode_packet(pkt, wire.data(), wire.size(), wire_size));

    // Corrupt CRC
    wire[wire_size - 1] ^= 0xFF;

    PacketParser parser;
    Packet decoded;
    bool parsed = false;
    for (size_t i = 0; i < wire_size; ++i) {
        parsed = parser.process_byte(wire[i], decoded);
    }
    EXPECT_FALSE(parsed);
    EXPECT_EQ(parser.crc_errors(), 1);
    EXPECT_EQ(parser.state(), ParserState::WAIT_SOF1);
}

TEST(ProtocolTest, InterByteTimeoutResets) {
    PacketParser parser;
    Packet decoded;

    parser.process_byte(SOF1, decoded);
    parser.process_byte(SOF2, decoded);
    parser.process_byte(0x05, decoded);
    EXPECT_NE(parser.state(), ParserState::WAIT_SOF1);

    bool reset = parser.tick(60);
    EXPECT_TRUE(reset);
    EXPECT_EQ(parser.state(), ParserState::WAIT_SOF1);
    EXPECT_EQ(parser.timeout_errors(), 1);
}

TEST(ProtocolTest, FuzzParserWithRandomGarbageThenRecover) {
    PacketParser parser;
    Packet decoded;

    std::mt19937 rng(1337);
    std::uniform_int_distribution<int> dist(0, 255);

    // Feed 10,000 random bytes
    for (int i = 0; i < 10000; ++i) {
        uint8_t b = static_cast<uint8_t>(dist(rng));
        // We do not expect a crash or out-of-bounds access
        parser.process_byte(b, decoded);
    }

    // Now send a valid HELLO packet
    Packet valid_hello;
    valid_hello.seq = 42;
    valid_hello.cmd = static_cast<uint8_t>(Command::HELLO);
    valid_hello.payload_len = 3;
    valid_hello.payload[0] = 1;
    valid_hello.payload[1] = 0;
    valid_hello.payload[2] = 100;

    std::array<uint8_t, MAX_PACKET_SIZE> wire{};
    size_t wire_size = 0;
    ASSERT_TRUE(encode_packet(valid_hello, wire.data(), wire.size(), wire_size));

    bool recovered = false;
    for (size_t i = 0; i < wire_size; ++i) {
        if (parser.process_byte(wire[i], decoded)) {
            recovered = true;
            break;
        }
    }
    EXPECT_TRUE(recovered);
    EXPECT_EQ(decoded.seq, 42);
    EXPECT_EQ(decoded.cmd, static_cast<uint8_t>(Command::HELLO));
}
