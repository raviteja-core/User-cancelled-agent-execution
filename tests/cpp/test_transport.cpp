#include <gtest/gtest.h>
#include "transport_mem.hpp"
#include "core/protocol/encoder.hpp"
#include "core/protocol/parser.hpp"

using namespace papercutter;
using namespace papercutter::protocol;

TEST(TransportTest, MemoryTransportPairEcho) {
    auto [ctrl_transport, test_transport] = create_memory_transport_pair();
    ASSERT_TRUE(ctrl_transport->is_open());
    ASSERT_TRUE(test_transport->is_open());

    // Send HELLO from test_transport to ctrl_transport
    Packet hello;
    hello.seq = 1;
    hello.cmd = static_cast<uint8_t>(Command::HELLO);
    hello.payload_len = 3;
    hello.payload[0] = 1;
    hello.payload[1] = 0;
    hello.payload[2] = 100;

    uint8_t tx_buf[MAX_PACKET_SIZE];
    size_t tx_size = 0;
    ASSERT_TRUE(encode_packet(hello, tx_buf, sizeof(tx_buf), tx_size));

    int written = test_transport->write(tx_buf, tx_size);
    EXPECT_EQ(written, static_cast<int>(tx_size));

    // Controller reads the data
    uint8_t rx_buf[256];
    int read_bytes = ctrl_transport->read(rx_buf, sizeof(rx_buf));
    EXPECT_EQ(read_bytes, static_cast<int>(tx_size));

    PacketParser parser;
    Packet rx_pkt;
    bool parsed = false;
    for (int i = 0; i < read_bytes; ++i) {
        if (parser.process_byte(rx_buf[i], rx_pkt)) {
            parsed = true;
            break;
        }
    }
    ASSERT_TRUE(parsed);
    EXPECT_EQ(rx_pkt.seq, 1);
    EXPECT_EQ(rx_pkt.cmd, static_cast<uint8_t>(Command::HELLO));

    // Controller replies with ACK (seq=10, acked_seq=1, status=0)
    size_t ack_size = 0;
    ASSERT_TRUE(encode_ack(10, rx_pkt.seq, 0, tx_buf, sizeof(tx_buf), ack_size));
    ctrl_transport->write(tx_buf, ack_size);

    // Test transport receives the ACK
    read_bytes = test_transport->read(rx_buf, sizeof(rx_buf));
    EXPECT_EQ(read_bytes, static_cast<int>(ack_size));

    PacketParser test_parser;
    Packet ack_pkt;
    parsed = false;
    for (int i = 0; i < read_bytes; ++i) {
        if (test_parser.process_byte(rx_buf[i], ack_pkt)) {
            parsed = true;
            break;
        }
    }
    ASSERT_TRUE(parsed);
    EXPECT_EQ(ack_pkt.cmd, static_cast<uint8_t>(Command::ACK));
    EXPECT_EQ(ack_pkt.payload[0], 1); // acked seq
    EXPECT_EQ(ack_pkt.payload[1], 0); // status ok
}

TEST(TransportTest, MemoryTransportClosedHandlesErrors) {
    auto [a, b] = create_memory_transport_pair();
    b->close();
    EXPECT_FALSE(b->is_open());

    uint8_t dummy[4] = {1, 2, 3, 4};
    int res = a->write(dummy, 4);
    EXPECT_EQ(res, -1);
}
