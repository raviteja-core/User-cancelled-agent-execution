#include "transport_serial_win.hpp"

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <iostream>

namespace papercutter {

WinSerialTransport::WinSerialTransport() noexcept : handle_(INVALID_HANDLE_VALUE) {}

WinSerialTransport::~WinSerialTransport() {
    close();
}

bool WinSerialTransport::open(const std::string& port_name, uint32_t baud_rate) noexcept {
    close();

    std::wstring full_path;
    if (port_name.rfind(R"(\\.\)", 0) == 0) {
        full_path = std::wstring(port_name.begin(), port_name.end());
    } else {
        full_path = L"\\\\.\\" + std::wstring(port_name.begin(), port_name.end());
    }

    HANDLE h = CreateFileW(
        full_path.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0,              // Must be opened with exclusive-access
        nullptr,        // Default security attributes
        OPEN_EXISTING,  // Must use OPEN_EXISTING
        0,              // Non-overlapped I/O
        nullptr         // hTemplate must be NULL for comm devices
    );

    if (h == INVALID_HANDLE_VALUE) {
        return false;
    }

    DCB dcb{};
    dcb.DCBlength = sizeof(DCB);
    if (!GetCommState(h, &dcb)) {
        CloseHandle(h);
        return false;
    }

    dcb.BaudRate = baud_rate;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
    dcb.fRtsControl = RTS_CONTROL_ENABLE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDsrSensitivity = FALSE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;

    if (!SetCommState(h, &dcb)) {
        CloseHandle(h);
        return false;
    }

    // Configure non-blocking timeouts: return immediately if no data
    COMMTIMEOUTS timeouts{};
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = 0;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 100; // 100 ms max write timeout

    if (!SetCommTimeouts(h, &timeouts)) {
        CloseHandle(h);
        return false;
    }

    PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);
    handle_ = h;
    return true;
}

int WinSerialTransport::read(uint8_t* buf, size_t capacity) noexcept {
    if (!is_open() || buf == nullptr || capacity == 0) {
        return -1;
    }

    DWORD bytes_read = 0;
    BOOL ok = ReadFile(
        static_cast<HANDLE>(handle_),
        buf,
        static_cast<DWORD>(capacity),
        &bytes_read,
        nullptr
    );

    if (!ok) {
        return -1;
    }
    return static_cast<int>(bytes_read);
}

int WinSerialTransport::write(const uint8_t* buf, size_t size) noexcept {
    if (!is_open() || buf == nullptr) {
        return -1;
    }
    if (size == 0) {
        return 0;
    }

    DWORD bytes_written = 0;
    BOOL ok = WriteFile(
        static_cast<HANDLE>(handle_),
        buf,
        static_cast<DWORD>(size),
        &bytes_written,
        nullptr
    );

    if (!ok) {
        return -1;
    }
    return static_cast<int>(bytes_written);
}

bool WinSerialTransport::is_open() const noexcept {
    return handle_ != nullptr && handle_ != INVALID_HANDLE_VALUE;
}

void WinSerialTransport::close() noexcept {
    if (is_open()) {
        CloseHandle(static_cast<HANDLE>(handle_));
        handle_ = INVALID_HANDLE_VALUE;
    }
}

} // namespace papercutter
