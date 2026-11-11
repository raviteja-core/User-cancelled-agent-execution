#include "transport_mem.hpp"
#include <algorithm>

namespace papercutter {

void MemoryTransport::connect_peer(MemoryTransport* peer) noexcept {
    peer_ = peer;
    if (peer != nullptr && peer->peer_ != this) {
        peer->peer_ = this;
    }
}

int MemoryTransport::read(uint8_t* buf, size_t capacity) noexcept {
    if (!is_open_) {
        return -1;
    }
    if (buf == nullptr || capacity == 0 || rx_count_ == 0) {
        return 0;
    }

    size_t to_read = std::min(capacity, rx_count_);
    for (size_t i = 0; i < to_read; ++i) {
        buf[i] = rx_ring_[rx_tail_];
        rx_tail_ = (rx_tail_ + 1) % BUFFER_SIZE;
    }
    rx_count_ -= to_read;
    return static_cast<int>(to_read);
}

int MemoryTransport::write(const uint8_t* buf, size_t size) noexcept {
    if (!is_open_) {
        return -1;
    }
    if (buf == nullptr || size == 0) {
        return 0;
    }
    if (peer_ == nullptr || !peer_->is_open_) {
        return -1; // Peer disconnected
    }

    bool ok = peer_->push_rx(buf, size);
    return ok ? static_cast<int>(size) : -1;
}

bool MemoryTransport::is_open() const noexcept {
    return is_open_;
}

void MemoryTransport::close() noexcept {
    is_open_ = false;
    rx_count_ = 0;
    rx_head_ = 0;
    rx_tail_ = 0;
}

bool MemoryTransport::push_rx(const uint8_t* data, size_t len) noexcept {
    if (!is_open_ || data == nullptr) {
        return false;
    }
    if (len > (BUFFER_SIZE - rx_count_)) {
        return false; // Overflow protection
    }

    for (size_t i = 0; i < len; ++i) {
        rx_ring_[rx_head_] = data[i];
        rx_head_ = (rx_head_ + 1) % BUFFER_SIZE;
    }
    rx_count_ += len;
    return true;
}

size_t MemoryTransport::rx_available() const noexcept {
    return rx_count_;
}

std::pair<std::unique_ptr<MemoryTransport>, std::unique_ptr<MemoryTransport>> create_memory_transport_pair() {
    auto a = std::make_unique<MemoryTransport>();
    auto b = std::make_unique<MemoryTransport>();
    a->connect_peer(b.get());
    return {std::move(a), std::move(b)};
}

} // namespace papercutter
