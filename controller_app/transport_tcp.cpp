#include "transport_tcp.hpp"

#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>

#pragma comment(lib, "ws2_32.lib")

namespace papercutter {

namespace {

struct WsaInit {
    WsaInit() {
        WSADATA data;
        WSAStartup(MAKEWORD(2, 2), &data);
    }
    ~WsaInit() {
        WSACleanup();
    }
};

static WsaInit g_wsa_init;

} // namespace

TcpTransport::TcpTransport() noexcept : listen_sock_(INVALID_SOCKET), client_sock_(INVALID_SOCKET) {}

TcpTransport::~TcpTransport() {
    close();
}

bool TcpTransport::listen_and_accept(const std::string& host, uint16_t port, bool non_blocking) noexcept {
    close();
    is_non_blocking_ = non_blocking;

    SOCKET s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (s == INVALID_SOCKET) {
        return false;
    }

    int opt = 1;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&opt), sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    inet_pton(AF_INET, host.c_str(), &addr.sin_addr);

    if (bind(s, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == SOCKET_ERROR) {
        closesocket(s);
        return false;
    }

    if (listen(s, 1) == SOCKET_ERROR) {
        closesocket(s);
        return false;
    }

    listen_sock_ = s;

    // Wait for client connection
    sockaddr_in client_addr{};
    int client_addr_len = sizeof(client_addr);
    SOCKET client = accept(s, reinterpret_cast<sockaddr*>(&client_addr), &client_addr_len);
    if (client == INVALID_SOCKET) {
        return false;
    }

    if (non_blocking) {
        u_long mode = 1;
        ioctlsocket(client, FIONBIO, &mode);
    }

    client_sock_ = client;
    return true;
}

bool TcpTransport::connect_to(const std::string& host, uint16_t port) noexcept {
    close();

    SOCKET s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (s == INVALID_SOCKET) {
        return false;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    inet_pton(AF_INET, host.c_str(), &addr.sin_addr);

    if (connect(s, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == SOCKET_ERROR) {
        closesocket(s);
        return false;
    }

    if (is_non_blocking_) {
        u_long mode = 1;
        ioctlsocket(s, FIONBIO, &mode);
    }

    client_sock_ = s;
    return true;
}

int TcpTransport::read(uint8_t* buf, size_t capacity) noexcept {
    if (!is_open() || buf == nullptr || capacity == 0) {
        return -1;
    }

    int res = recv(static_cast<SOCKET>(client_sock_), reinterpret_cast<char*>(buf), static_cast<int>(capacity), 0);
    if (res > 0) {
        return res;
    }
    if (res == 0) {
        return -1; // Graceful disconnect
    }

    int err = WSAGetLastError();
    if (err == WSAEWOULDBLOCK) {
        return 0; // No data yet (non-blocking)
    }
    return -1; // Socket error
}

int TcpTransport::write(const uint8_t* buf, size_t size) noexcept {
    if (!is_open() || buf == nullptr) {
        return -1;
    }
    if (size == 0) {
        return 0;
    }

    int res = send(static_cast<SOCKET>(client_sock_), reinterpret_cast<const char*>(buf), static_cast<int>(size), 0);
    if (res >= 0) {
        return res;
    }

    int err = WSAGetLastError();
    if (err == WSAEWOULDBLOCK) {
        return 0;
    }
    return -1;
}

bool TcpTransport::is_open() const noexcept {
    return client_sock_ != static_cast<uint64_t>(INVALID_SOCKET);
}

void TcpTransport::close() noexcept {
    if (client_sock_ != static_cast<uint64_t>(INVALID_SOCKET)) {
        closesocket(static_cast<SOCKET>(client_sock_));
        client_sock_ = INVALID_SOCKET;
    }
    if (listen_sock_ != static_cast<uint64_t>(INVALID_SOCKET)) {
        closesocket(static_cast<SOCKET>(listen_sock_));
        listen_sock_ = INVALID_SOCKET;
    }
}

} // namespace papercutter
