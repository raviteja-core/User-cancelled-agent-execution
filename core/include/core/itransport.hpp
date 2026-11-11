#pragma once

#include <cstddef>
#include <cstdint>

namespace papercutter {

class ITransport {
public:
    virtual ~ITransport() = default;

    // Reads up to capacity bytes into buf. Returns number of bytes read (0 if no data available, -1 on error).
    virtual int read(uint8_t* buf, size_t capacity) noexcept = 0;

    // Writes size bytes from buf. Returns number of bytes written (-1 on error).
    virtual int write(const uint8_t* buf, size_t size) noexcept = 0;

    // Returns true if transport is open and ready.
    [[nodiscard]] virtual bool is_open() const noexcept = 0;
};

} // namespace papercutter
