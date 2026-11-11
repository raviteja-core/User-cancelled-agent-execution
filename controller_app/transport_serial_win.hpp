#pragma once

#include "core/itransport.hpp"
#include <string>

namespace papercutter {

class WinSerialTransport : public ITransport {
public:
    WinSerialTransport() noexcept;
    ~WinSerialTransport() override;

    // Opens a COM port, e.g. "COM10" or "\\\\.\\COM10", with specified baud (default 115200 8N1)
    bool open(const std::string& port_name, uint32_t baud_rate = 115200) noexcept;

    int read(uint8_t* buf, size_t capacity) noexcept override;
    int write(const uint8_t* buf, size_t size) noexcept override;
    [[nodiscard]] bool is_open() const noexcept override;
    void close() noexcept override;

private:
    void* handle_{nullptr}; // HANDLE (stored as void* to keep header lightweight)
};

} // namespace papercutter
