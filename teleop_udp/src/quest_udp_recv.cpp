
/*
 * =====================================================================================
 *
 * Filename:  quest_udp_recv.cpp
 *
 * Description:  The implementation for your UDP receiver class.
 *
 * =====================================================================================
 */
#include "teleop_udp/quest_udp_recv.hpp"
#include <iostream>
#include <cstring> // For memset

#ifndef _WIN32
    #include <cerrno> // For errno, EWOULDBLOCK, EAGAIN
#endif

QuestUdpReceiver::QuestUdpReceiver(int port) : m_socket_fd(-1) {
    #ifdef _WIN32
        m_socket_fd = INVALID_SOCKET;
    #else
        m_socket_fd = -1;
    #endif
    
    if (!init_socket(port)) {
        std::cerr << "Error: Failed to initialize UDP receiver socket." << std::endl;
    }
}

QuestUdpReceiver::~QuestUdpReceiver() {
    #ifdef _WIN32
        if (m_socket_fd != INVALID_SOCKET) {
            closesocket(m_socket_fd);
            WSACleanup();
        }
    #else
        if (m_socket_fd != -1) {
            close(m_socket_fd);
        }
    #endif
}

bool QuestUdpReceiver::init_socket(int port) {
    #ifdef _WIN32
        if (WSAStartup(MAKEWORD(2, 2), &m_wsa_data) != 0) {
            std::cerr << "WSAStartup failed." << std::endl;
            return false;
        }
    #endif

    m_socket_fd = socket(AF_INET, SOCK_DGRAM, 0);

    #ifdef _WIN32
        if (m_socket_fd == INVALID_SOCKET) {
            std::cerr << "Error: Cannot create receiver socket." << std::endl;
            return false;
        }
    #else
        if (m_socket_fd < 0) {
            std::cerr << "Error: Cannot create receiver socket." << std::endl;
            return false;
        }
    #endif

    // --- Set socket to non-blocking ---
    #ifdef _WIN32
        unsigned long mode = 1; // 1 for non-blocking
        if (ioctlsocket(m_socket_fd, FIONBIO, &mode) != 0) {
            std::cerr << "Error: Failed to set non-blocking socket on Windows." << std::endl;
            closesocket(m_socket_fd);
            m_socket_fd = INVALID_SOCKET;
            return false;
        }
    #else
        int flags = fcntl(m_socket_fd, F_GETFL, 0);
        if (flags < 0) {
            std::cerr << "Error: Failed to get socket flags on Linux." << std::endl;
            close(m_socket_fd);
            m_socket_fd = -1;
            return false;
        }
        if (fcntl(m_socket_fd, F_SETFL, flags | O_NONBLOCK) < 0) {
            std::cerr << "Error: Failed to set non-blocking socket on Linux." << std::endl;
            close(m_socket_fd);
            m_socket_fd = -1;
            return false;
        }
    #endif

    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY; // Listen on all available network interfaces
    server_addr.sin_port = htons(port);

    // Bind the socket to the specified port
    if (bind(m_socket_fd, (const struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "Error: Failed to bind receiver socket to port " << port << std::endl;
        #ifdef _WIN32
            closesocket(m_socket_fd);
            m_socket_fd = INVALID_SOCKET;
        #else
            close(m_socket_fd);
            m_socket_fd = -1;
        #endif
        return false;
    }

    std::cout << "UDP Receiver initialized. Listening on port " << port << std::endl;
    return true;
}

bool QuestUdpReceiver::receiveData(JoystickData& data) {
    #ifdef _WIN32
        if (m_socket_fd == INVALID_SOCKET) return false;
        // For Windows, recvfrom length argument is int
        int client_len = sizeof(struct sockaddr_in);
    #else
        if (m_socket_fd < 0) return false;
        // For Linux, it's socklen_t
        socklen_t client_len = sizeof(struct sockaddr_in);
    #endif

    struct sockaddr_in client_addr;
    
    // Receive data from the network
    // ssize_t is not defined on Windows, use int for recvfrom return value
    #ifdef _WIN32
        // ssize_t is not defined on Windows, use int for recvfrom return value
        int bytes_received = recvfrom(m_socket_fd, 
                                    reinterpret_cast<char*>(&data), 
                                    sizeof(JoystickData), 
                                    0, 
                                    (struct sockaddr*)&client_addr, 
                                    &client_len);
    #else
        ssize_t bytes_received = recvfrom(m_socket_fd, 
                                        reinterpret_cast<char*>(&data), 
                                        sizeof(JoystickData), 
                                        0, 
                                        (struct sockaddr*)&client_addr, 
                                        &client_len);
    #endif
    
    if (bytes_received < 0) {
        // On a non-blocking socket, an error can just mean "no data yet".
        #ifdef _WIN32
            if (WSAGetLastError() == WSAEWOULDBLOCK) {
                return false; // This is expected when no data is available
            }
        #else
            if (errno == EWOULDBLOCK || errno == EAGAIN) {
                return false; // This is expected when no data is available
            }
        #endif
        // If it's a different error, something is wrong.
        // std::cerr << "Error: recvfrom failed with a real network error." << std::endl;
        return false;
    }

    return bytes_received == sizeof(JoystickData); // No data or incomplete data received
}
