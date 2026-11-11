#pragma once

#include "core/itransport.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>

namespace papercutter {

class MemoryTransport : public ITransport {
public:
    static constexpr size_t BUFFER_SIZE = 4096;

    MemoryTransport() noexcept = default;
    ~MemoryTransport() override = default;

    // Connects this transport to a peer transport.
    // What this writes will be read by the peer, and vice versa.
    void connect_peer(MemoryTransport* peer) noexcept;

    int read(uint8_t* buf, size_t capacity) noexcept override;
    int write(const uint8_t* buf, size_t size) noexcept override;
    [[nodiscard]] bool is_open() const noexcept override;
    void close() noexcept override;

    // Helper for direct pushing into receive buffer (used in unit tests)
    bool push_rx(const uint8_t* data, size_t len) noexcept;

    // Helper to inspect receive buffer size
    [[nodiscard]] size_t rx_available() const noexcept;

private:
    MemoryTransport* peer_{nullptr};
    std::array<uint8_t, BUFFER_SIZE> rx_ring_{};
    size_t rx_head_{0};
    size_t rx_tail_{0};
    size_t rx_count_{0};
    bool is_open_{true};
};

// Factory to create a connected pair
std::pair<std::unique_ptr<MemoryTransport>, std::unique_ptr<MemoryTransport>> create_memory_transport_pair();

} // namespace papercutter
