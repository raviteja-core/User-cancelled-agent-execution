#pragma once

#include "core/itransport.hpp"
#include <cstdint>
#include <string>

namespace papercutter {

class TcpTransport : public ITransport {
public:
    TcpTransport() noexcept;
    ~TcpTransport() override;

    // Listens on host:port (e.g. "127.0.0.1", 9000) for a client to connect.
    // If wait_for_client is true, blocks until a client connects or timeout.
    bool listen_and_accept(const std::string& host, uint16_t port, bool non_blocking = true) noexcept;

    // Connects to a remote server (client mode)
    bool connect_to(const std::string& host, uint16_t port) noexcept;

    int read(uint8_t* buf, size_t capacity) noexcept override;
    int write(const uint8_t* buf, size_t size) noexcept override;
    [[nodiscard]] bool is_open() const noexcept override;
    void close() noexcept override;

private:
    uint64_t listen_sock_{~0ULL}; // SOCKET
    uint64_t client_sock_{~0ULL}; // SOCKET
    bool is_non_blocking_{true};
};

} // namespace papercutter
