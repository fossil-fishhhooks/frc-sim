#pragma once
#include <cstdio>
#include <atomic>
#include <mutex>
#include <thread>
#include <vector>

#ifdef _WIN32
#  include <winsock2.h>
   using SOCK_T = SOCKET;
#  define BAD_SOCK INVALID_SOCKET
#else
   using SOCK_T = int;
#  define BAD_SOCK (-1)
#endif

class StreamEncoder
{
public:
    // Start encoding + HTTP server. Returns false on error.
    bool Init(int port, int width, int height, int fps);

    // Push one RGBA frame. width/height must match Init().
    void PushFrame(const void *rgba_data, int width, int height);

    // Stop everything cleanly.
    void Shutdown();

    bool IsRunning() const { return m_running && m_pipe; }

private:
    // ffmpeg I/O
    FILE *m_pipe    = nullptr;   // write → ffmpeg stdin
    int   m_read_fd = -1;        // read  ← ffmpeg stdout (raw fd)

    // HTTP fanout server
    int                  m_server_fd = (int)BAD_SOCK;
    std::mutex           m_clients_mtx;
    std::vector<SOCK_T>  m_clients;

    // Background threads
    std::thread          m_read_thread;
    std::thread          m_accept_thread;
    std::atomic<bool>    m_running{false};

    int m_width  = 0;
    int m_height = 0;
};