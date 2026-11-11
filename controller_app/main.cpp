#include "core/protocol/packet.hpp"
#include "core/protocol/encoder.hpp"
#include "core/protocol/parser.hpp"
#include "transport_serial_win.hpp"
#include "transport_tcp.hpp"

#include <iostream>
#include <string>
#include <memory>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>

using namespace papercutter;
using namespace papercutter::protocol;

static std::atomic<bool> g_running{true};

static void sig_handler(int) {
    g_running = false;
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT, sig_handler);
    std::signal(SIGTERM, sig_handler);

    std::string serial_port = "COM10";
    uint32_t baud_rate = 115200;
    uint16_t tcp_port = 0;
    bool use_tcp = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--serial" && i + 1 < argc) {
            serial_port = argv[++i];
            use_tcp = false;
        } else if (arg == "--baud" && i + 1 < argc) {
            baud_rate = static_cast<uint32_t>(std::stoul(argv[++i]));
        } else if (arg == "--tcp" && i + 1 < argc) {
            tcp_port = static_cast<uint16_t>(std::stoul(argv[++i]));
            use_tcp = true;
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: controller_app [options]\n"
                      << "Options:\n"
                      << "  --serial <port>  Use Win32 COM port (default: COM10)\n"
                      << "  --baud <rate>    Serial baud rate (default: 115200)\n"
                      << "  --tcp <port>     Use TCP server on localhost:<port>\n"
                      << "  --help, -h       Show help\n";
            return 0;
        }
    }

    std::unique_ptr<ITransport> transport;

    if (use_tcp) {
        std::cout << "[Controller] Starting in TCP mode on 127.0.0.1:" << tcp_port << "...\n";
        auto tcp = std::make_unique<TcpTransport>();
        if (!tcp->listen_and_accept("127.0.0.1", tcp_port, true)) {
            std::cerr << "[Controller] Failed to bind/listen/accept on TCP port " << tcp_port << "\n";
            return 1;
        }
        std::cout << "[Controller] Client connected on TCP.\n";
        transport = std::move(tcp);
    } else {
        std::cout << "[Controller] Opening serial port " << serial_port << " at " << baud_rate << " baud...\n";
        auto serial = std::make_unique<WinSerialTransport>();
        if (!serial->open(serial_port, baud_rate)) {
            std::cerr << "[Controller] Could not open serial port " << serial_port << ".\n";
            std::cerr << "[Controller] (Ensure com0com virtual COM pair COM10-COM11 is configured)\n";
            return 1;
        }
        std::cout << "[Controller] Serial port " << serial_port << " opened.\n";
        transport = std::move(serial);
    }

    PacketParser parser;
    Packet rx_packet;
    uint8_t rx_buf[256];
    uint8_t tx_buf[MAX_PACKET_SIZE];
    uint8_t tx_seq = 0;

    std::cout << "[Controller] Running 1 ms control tick loop (Ctrl+C to stop)...\n";

    auto next_tick = std::chrono::steady_clock::now();

    while (g_running && transport->is_open()) {
        auto now = std::chrono::steady_clock::now();
        if (now < next_tick) {
            std::this_thread::sleep_until(next_tick);
        }
        next_tick += std::chrono::milliseconds(1);

        // 1 ms tick update
        parser.tick(1);

        // Read available bytes
        int bytes_read = transport->read(rx_buf, sizeof(rx_buf));
        if (bytes_read > 0) {
            for (int i = 0; i < bytes_read; ++i) {
                if (parser.process_byte(rx_buf[i], rx_packet)) {
                    // Packet successfully received
                    size_t tx_size = 0;

                    if (rx_packet.cmd == static_cast<uint8_t>(Command::HELLO)) {
                        // Respond with HELLO (proto=1, fw=100)
                        Packet hello_resp;
                        hello_resp.seq = tx_seq++;
                        hello_resp.cmd = static_cast<uint8_t>(Command::HELLO);
                        hello_resp.payload_len = 3;
                        hello_resp.payload[0] = 1;
                        hello_resp.payload[1] = 0;
                        hello_resp.payload[2] = 100;
                        if (encode_packet(hello_resp, tx_buf, sizeof(tx_buf), tx_size)) {
                            transport->write(tx_buf, tx_size);
                        }
                    }

                    // Send ACK for the received command
                    if (rx_packet.cmd != static_cast<uint8_t>(Command::ACK) &&
                        rx_packet.cmd != static_cast<uint8_t>(Command::NACK) &&
                        rx_packet.cmd != static_cast<uint8_t>(Command::TELEMETRY)) {
                        if (encode_ack(tx_seq++, rx_packet.seq, 0, tx_buf, sizeof(tx_buf), tx_size)) {
                            transport->write(tx_buf, tx_size);
                        }
                    }
                }
            }
        }
    }

    std::cout << "[Controller] Exiting safely.\n";
    transport->close();
    return 0;
}
