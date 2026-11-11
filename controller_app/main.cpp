#include "core/protocol/packet.hpp"
#include "core/protocol/encoder.hpp"
#include "core/protocol/parser.hpp"
#include "core/fsm/state_machine.hpp"
#include "core/motion/planner.hpp"
#include "core/sim/sim_hardware.hpp"
#include "core/safety/safety.hpp"
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
using namespace papercutter::fsm;
using namespace papercutter::motion;
using namespace papercutter::sim;

static std::atomic<bool> g_running{true};

static void sig_handler(int) {
    g_running = false;
}

static inline uint16_t read_u16_be(const uint8_t* p) {
    return static_cast<uint16_t>((p[0] << 8) | p[1]);
}

static inline uint32_t read_u32_be(const uint8_t* p) {
    return (static_cast<uint32_t>(p[0]) << 24) |
           (static_cast<uint32_t>(p[1]) << 16) |
           (static_cast<uint32_t>(p[2]) << 8) |
           static_cast<uint32_t>(p[3]);
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

    StateMachine fsm;
    TrapezoidalPlanner back_gauge;
    BladePlanner blade;
    SimulatedHardware sim;
    PacketParser parser;
    safety::SafetySupervisor safety;

    Packet rx_packet;
    uint8_t rx_buf[256];
    uint8_t tx_buf[MAX_PACKET_SIZE];
    uint8_t tx_seq = 0;

    bool client_connected = false;
    uint32_t heartbeat_watchdog_ms = 0;
    uint32_t telemetry_timer_ms = 0;

    // Fault injection variables
    uint16_t drop_next_packets = 0;
    uint16_t corrupt_crc_packets = 0;
    bool ignore_heartbeats = false;

    std::cout << "[Controller] Running 1 ms control tick loop (Ctrl+C to stop)...\n";

    auto next_tick = std::chrono::steady_clock::now();

    while (g_running && transport->is_open()) {
        auto now = std::chrono::steady_clock::now();
        if (now < next_tick) {
            std::this_thread::sleep_until(next_tick);
        }
        next_tick += std::chrono::milliseconds(1);

        // 1. Tick simulated hardware and motion
        sim.tick(1, fsm, back_gauge, blade);

        // 2. Tick parser timeout
        parser.tick(1);

        // 3. Safety Supervisor (heartbeat watchdog, interlocks, CRC storm, motion overrun)
        if (!ignore_heartbeats) {
            safety.tick_1ms(fsm, back_gauge, blade, parser.crc_errors(), client_connected);
        } else if (client_connected) {
            // Heartbeats ignored: watchdog will trip
            safety.tick_1ms(fsm, back_gauge, blade, parser.crc_errors(), true);
        }

        // 4. 20 Hz Telemetry broadcast (every 50 ms)
        telemetry_timer_ms += 1;
        if (telemetry_timer_ms >= 50) {
            telemetry_timer_ms = 0;
            if (client_connected) {
                auto telem = sim.generate_telemetry(
                    fsm,
                    back_gauge,
                    blade,
                    parser.crc_errors(),
                    parser.dropped_packets(),
                    parser.timeout_errors()
                );
                size_t telem_size = 0;
                if (encode_telemetry(telem, tx_seq++, tx_buf, sizeof(tx_buf), telem_size)) {
                    transport->write(tx_buf, telem_size);
                }
            }
        }

        // 5. Read incoming commands
        int bytes_read = transport->read(rx_buf, sizeof(rx_buf));
        if (bytes_read > 0) {
            for (int i = 0; i < bytes_read; ++i) {
                if (parser.process_byte(rx_buf[i], rx_packet)) {
                    // Packet received
                    if (drop_next_packets > 0) {
                        drop_next_packets--;
                        continue;
                    }

                    // Reset heartbeat watchdog on valid command
                    if (!ignore_heartbeats) {
                        safety.pet_watchdog();
                    }

                    size_t tx_size = 0;

                    switch (static_cast<Command>(rx_packet.cmd)) {
                        case Command::HELLO: {
                            client_connected = true;
                            heartbeat_watchdog_ms = 0;
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
                            encode_ack(tx_seq++, rx_packet.seq, 0, tx_buf, sizeof(tx_buf), tx_size);
                            transport->write(tx_buf, tx_size);
                            break;
                        }

                        case Command::HEARTBEAT: {
                            if (!ignore_heartbeats) {
                                encode_ack(tx_seq++, rx_packet.seq, 0, tx_buf, sizeof(tx_buf), tx_size);
                                transport->write(tx_buf, tx_size);
                            }
                            break;
                        }

                        case Command::LOAD_JOB: {
                            if (rx_packet.payload_len < 3) {
                                encode_nack(tx_seq++, rx_packet.seq, static_cast<uint8_t>(FaultCode::INVALID_JOB), tx_buf, sizeof(tx_buf), tx_size);
                                transport->write(tx_buf, tx_size);
                                break;
                            }
                            uint16_t job_id = read_u16_be(&rx_packet.payload[0]);
                            uint8_t count = rx_packet.payload[2];
                            if (count == 0 || rx_packet.payload_len < (3 + count * 4)) {
                                encode_nack(tx_seq++, rx_packet.seq, static_cast<uint8_t>(FaultCode::INVALID_JOB), tx_buf, sizeof(tx_buf), tx_size);
                                transport->write(tx_buf, tx_size);
                                break;
                            }

                            fsm.context().job_id = job_id;
                            fsm.context().total_cuts = count;
                            fsm.context().current_cut = 0;
                            for (uint8_t c = 0; c < count; ++c) {
                                fsm.context().cut_positions[c] = read_u32_be(&rx_packet.payload[3 + c * 4]);
                            }
                            fsm.context().job_loaded = true;

                            encode_ack(tx_seq++, rx_packet.seq, 0, tx_buf, sizeof(tx_buf), tx_size);
                            transport->write(tx_buf, tx_size);
                            break;
                        }

                        case Command::START: {
                            if (fsm.state() == State::ERROR_FAULT || fsm.state() == State::ERROR_JAM) {
                                encode_nack(tx_seq++, rx_packet.seq, static_cast<uint8_t>(fsm.context().latched_fault), tx_buf, sizeof(tx_buf), tx_size);
                                transport->write(tx_buf, tx_size);
                                break;
                            }

                            auto res = fsm.handle_event(Event::START);
                            if (res == TransitionResult::SUCCESS) {
                                uint32_t target = fsm.context().cut_positions[0];
                                back_gauge.plan_move(static_cast<int32_t>(target));
                                encode_ack(tx_seq++, rx_packet.seq, 0, tx_buf, sizeof(tx_buf), tx_size);
                            } else {
                                encode_nack(tx_seq++, rx_packet.seq, 1, tx_buf, sizeof(tx_buf), tx_size);
                            }
                            transport->write(tx_buf, tx_size);
                            break;
                        }

                        case Command::PAUSE_RESUME: {
                            encode_ack(tx_seq++, rx_packet.seq, 0, tx_buf, sizeof(tx_buf), tx_size);
                            transport->write(tx_buf, tx_size);
                            break;
                        }

                        case Command::ABORT: {
                            fsm.handle_event(Event::ABORT);
                            back_gauge.hard_stop();
                            blade.hard_stop();
                            encode_ack(tx_seq++, rx_packet.seq, 0, tx_buf, sizeof(tx_buf), tx_size);
                            transport->write(tx_buf, tx_size);
                            break;
                        }

                        case Command::ESTOP: {
                            // Immediate halt within 1 ms
                            fsm.force_estop();
                            back_gauge.emergency_stop();
                            blade.emergency_stop();
                            encode_ack(tx_seq++, rx_packet.seq, 0, tx_buf, sizeof(tx_buf), tx_size);
                            transport->write(tx_buf, tx_size);
                            break;
                        }

                        case Command::RESET_FAULT: {
                            // Only valid if physical sensors safe
                            auto res = fsm.handle_event(Event::RESET_FAULT);
                            if (res == TransitionResult::SUCCESS) {
                                sim.inject_paper_jam(false);
                                sim.inject_premature_blade_limit(false);
                                sim.inject_interlock_trip(false);
                                encode_ack(tx_seq++, rx_packet.seq, 0, tx_buf, sizeof(tx_buf), tx_size);
                            } else {
                                encode_nack(tx_seq++, rx_packet.seq, static_cast<uint8_t>(fsm.context().latched_fault), tx_buf, sizeof(tx_buf), tx_size);
                            }
                            transport->write(tx_buf, tx_size);
                            break;
                        }

                        case Command::SIM_INJECT: {
                            if (rx_packet.payload_len >= 3) {
                                uint8_t fault_type = rx_packet.payload[0];
                                uint16_t param = read_u16_be(&rx_packet.payload[1]);
                                if (fault_type == 1) { // Paper jam
                                    sim.inject_paper_jam(param != 0);
                                } else if (fault_type == 2) { // Comm timeout / stop heartbeats
                                    ignore_heartbeats = (param != 0);
                                } else if (fault_type == 3) { // Premature blade limit
                                    sim.inject_premature_blade_limit(param != 0);
                                } else if (fault_type == 4) { // E-stop
                                    fsm.force_estop();
                                    back_gauge.emergency_stop();
                                    blade.emergency_stop();
                                } else if (fault_type == 5) { // Drop next N packets
                                    drop_next_packets = param;
                                } else if (fault_type == 6) { // Corrupt CRC
                                    corrupt_crc_packets = param;
                                }
                            }
                            encode_ack(tx_seq++, rx_packet.seq, 0, tx_buf, sizeof(tx_buf), tx_size);
                            transport->write(tx_buf, tx_size);
                            break;
                        }

                        default: {
                            encode_nack(tx_seq++, rx_packet.seq, 0xFF, tx_buf, sizeof(tx_buf), tx_size);
                            transport->write(tx_buf, tx_size);
                            break;
                        }
                    }
                }
            }
        }
    }

    std::cout << "[Controller] Exiting cleanly.\n";
    transport->close();
    return 0;
}
