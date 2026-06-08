#pragma once
#include <cstdio>
#include <atomic>
#include <mutex>
#include <thread>
#include <vector>
#include <string>


#ifdef _WIN32
#  ifndef _WINSOCKAPI_   // guard: winsock2.h defines this
#    include <winsock2.h>
#  endif
   using sock_t = SOCKET;
#  define BAD_SOCK INVALID_SOCKET
#else
#  include <sys/types.h>
   using sock_t = int;
#  define BAD_SOCK (-1)
#endif

class StreamEncoder
{
public:
    bool Init(int port, int width, int height, int fps);
    void PushFrame(const void *rgba_data, int width, int height);
    void Shutdown();

    bool IsRunning() const { return m_running && m_pipe; }

private:
    FILE   *m_pipe    = nullptr;
    int     m_read_fd = -1;

#ifdef _WIN32
    bool    m_wsa_initialized = false;
#else
    pid_t   m_child_pid = -1;
#endif

    sock_t               m_server_fd = BAD_SOCK;
    std::mutex           m_clients_mtx;
    std::vector<sock_t>  m_clients;

    std::thread       m_read_thread;
    std::thread       m_accept_thread;
    std::atomic<bool> m_running{false};

    int m_width  = 0;
    int m_height = 0;
};