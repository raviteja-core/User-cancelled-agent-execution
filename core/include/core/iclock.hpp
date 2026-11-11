#pragma once

#include <cstdint>

namespace papercutter {

class IClock {
public:
    virtual ~IClock() = default;

    // Returns monotonic time in milliseconds
    [[nodiscard]] virtual uint64_t now_ms() const noexcept = 0;

    // Returns monotonic time in microseconds (for sub-millisecond motion control if needed)
    [[nodiscard]] virtual uint64_t now_us() const noexcept = 0;
};

} // namespace papercutter
