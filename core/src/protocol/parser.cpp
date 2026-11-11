#include "core/protocol/parser.hpp"
#include "core/protocol/crc16.hpp"

namespace papercutter::protocol {

void PacketParser::reset() noexcept {
    state_ = ParserState::WAIT_SOF1;
    expected_len_ = 0;
    body_bytes_read_ = 0;
    crc_high_ = 0;
    crc_low_ = 0;
    time_since_last_byte_ms_ = 0;
}

bool PacketParser::tick(uint32_t delta_ms) noexcept {
    if (state_ != ParserState::WAIT_SOF1) {
        time_since_last_byte_ms_ += delta_ms;
        if (time_since_last_byte_ms_ >= INTER_BYTE_TIMEOUT_MS) {
            ++timeout_errors_;
            reset();
            return true;
        }
    }
    return false;
}

bool PacketParser::process_byte(uint8_t byte, Packet& out_pkt) noexcept {
    time_since_last_byte_ms_ = 0;

    switch (state_) {
        case ParserState::WAIT_SOF1: {
            if (byte == SOF1) {
                state_ = ParserState::WAIT_SOF2;
            }
            return false;
        }

        case ParserState::WAIT_SOF2: {
            if (byte == SOF2) {
                state_ = ParserState::READ_LEN;
            } else if (byte == SOF1) {
                // If another 0xAA arrives, stay in WAIT_SOF2
                state_ = ParserState::WAIT_SOF2;
            } else {
                state_ = ParserState::WAIT_SOF1;
            }
            return false;
        }

        case ParserState::READ_LEN: {
            if (byte >= MIN_PAYLOAD_LEN && byte <= MAX_PAYLOAD_LEN) {
                expected_len_ = byte;
                body_bytes_read_ = 0;
                body_buf_[0] = byte; // Store LEN for CRC calculation
                state_ = ParserState::READ_BODY;
            } else {
                ++dropped_packets_;
                state_ = ParserState::WAIT_SOF1;
            }
            return false;
        }

        case ParserState::READ_BODY: {
            // Buffer boundary protection
            if (body_bytes_read_ < expected_len_) {
                body_buf_[1 + body_bytes_read_] = byte;
                ++body_bytes_read_;
            }

            if (body_bytes_read_ >= expected_len_) {
                state_ = ParserState::READ_CRC1;
            }
            return false;
        }

        case ParserState::READ_CRC1: {
            crc_high_ = byte;
            state_ = ParserState::READ_CRC2;
            return false;
        }

        case ParserState::READ_CRC2: {
            crc_low_ = byte;
            uint16_t received_crc = static_cast<uint16_t>((static_cast<uint16_t>(crc_high_) << 8) | crc_low_);

            // CRC is computed over body_buf_[0 .. expected_len_] (which contains LEN, SEQ, CMD, PAYLOAD)
            uint16_t calculated_crc = crc16_ccitt_false(&body_buf_[0], 1 + expected_len_);

            if (received_crc == calculated_crc) {
                out_pkt.seq = body_buf_[1];
                out_pkt.cmd = body_buf_[2];
                out_pkt.payload_len = static_cast<uint8_t>(expected_len_ - 2);

                for (size_t i = 0; i < out_pkt.payload_len; ++i) {
                    out_pkt.payload[i] = body_buf_[3 + i];
                }

                reset();
                return true;
            } else {
                ++crc_errors_;
                reset();
                return false;
            }
        }

        default: {
            reset();
            return false;
        }
    }
}

} // namespace papercutter::protocol
