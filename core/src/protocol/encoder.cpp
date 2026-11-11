#include "core/protocol/encoder.hpp"
#include "core/protocol/crc16.hpp"

namespace papercutter::protocol {

namespace {

inline void write_u16_be(uint8_t* dest, uint16_t val) noexcept {
    dest[0] = static_cast<uint8_t>((val >> 8) & 0xFF);
    dest[1] = static_cast<uint8_t>(val & 0xFF);
}

inline void write_i32_be(uint8_t* dest, int32_t val) noexcept {
    auto uval = static_cast<uint32_t>(val);
    dest[0] = static_cast<uint8_t>((uval >> 24) & 0xFF);
    dest[1] = static_cast<uint8_t>((uval >> 16) & 0xFF);
    dest[2] = static_cast<uint8_t>((uval >> 8) & 0xFF);
    dest[3] = static_cast<uint8_t>(uval & 0xFF);
}

} // namespace

bool encode_packet(const Packet& pkt, uint8_t* out_buf, size_t out_capacity, size_t& out_size) noexcept {
    out_size = 0;
    if (out_buf == nullptr) {
        return false;
    }

    size_t length = static_cast<size_t>(pkt.payload_len) + 2; // SEQ (1) + CMD (1) + payload
    if (length < MIN_PAYLOAD_LEN || length > MAX_PAYLOAD_LEN) {
        return false;
    }

    size_t total_wire_len = PREAMBLE_SIZE + pkt.payload_len + CRC_SIZE;
    if (out_capacity < total_wire_len) {
        return false;
    }

    out_buf[0] = SOF1;
    out_buf[1] = SOF2;
    out_buf[2] = static_cast<uint8_t>(length);
    out_buf[3] = pkt.seq;
    out_buf[4] = pkt.cmd;

    for (size_t i = 0; i < pkt.payload_len; ++i) {
        out_buf[5 + i] = pkt.payload[i];
    }

    // CRC is over LEN (out_buf[2]), SEQ (out_buf[3]), CMD (out_buf[4]), and PAYLOAD (out_buf[5..])
    // That is total of (1 + length) bytes starting at &out_buf[2]
    uint16_t crc = crc16_ccitt_false(&out_buf[2], 1 + length);

    size_t crc_offset = PREAMBLE_SIZE + pkt.payload_len;
    write_u16_be(&out_buf[crc_offset], crc);

    out_size = total_wire_len;
    return true;
}

bool encode_telemetry(const TelemetryPayload& telem, uint8_t seq, uint8_t* out_buf, size_t out_capacity, size_t& out_size) noexcept {
    Packet pkt;
    pkt.seq = seq;
    pkt.cmd = static_cast<uint8_t>(Command::TELEMETRY);
    pkt.payload_len = static_cast<uint8_t>(TELEMETRY_PAYLOAD_SIZE);

    pkt.payload[0] = telem.state;
    write_u16_be(&pkt.payload[1], telem.fault_code);
    write_i32_be(&pkt.payload[3], telem.back_gauge_pos);
    write_i32_be(&pkt.payload[7], telem.blade_pos);
    pkt.payload[11] = telem.feed_sensor;
    pkt.payload[12] = telem.limit_switches;
    pkt.payload[13] = telem.cut_index;
    write_u16_be(&pkt.payload[14], telem.crc_errors);
    write_u16_be(&pkt.payload[16], telem.dropped_packets);
    write_u16_be(&pkt.payload[18], telem.timeout_errors);
    pkt.payload[20] = telem.clamp_engaged;
    write_u16_be(&pkt.payload[21], telem.reserved);

    return encode_packet(pkt, out_buf, out_capacity, out_size);
}

bool encode_ack(uint8_t seq, uint8_t acked_seq, uint8_t status, uint8_t* out_buf, size_t out_capacity, size_t& out_size) noexcept {
    Packet pkt;
    pkt.seq = seq;
    pkt.cmd = static_cast<uint8_t>(Command::ACK);
    pkt.payload_len = 2;
    pkt.payload[0] = acked_seq;
    pkt.payload[1] = status;
    return encode_packet(pkt, out_buf, out_capacity, out_size);
}

bool encode_nack(uint8_t seq, uint8_t rejected_seq, uint8_t reason, uint8_t* out_buf, size_t out_capacity, size_t& out_size) noexcept {
    Packet pkt;
    pkt.seq = seq;
    pkt.cmd = static_cast<uint8_t>(Command::NACK);
    pkt.payload_len = 2;
    pkt.payload[0] = rejected_seq;
    pkt.payload[1] = reason;
    return encode_packet(pkt, out_buf, out_capacity, out_size);
}

} // namespace papercutter::protocol
