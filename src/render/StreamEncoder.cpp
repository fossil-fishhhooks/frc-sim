#include "render/StreamEncoder.h"
#include "io/EasyLog.h"
#include <cstring>
#include <initializer_list>
#include <thread>
#include <mutex>
#include <vector>
#include <atomic>
#include <algorithm>
#include <chrono>
#ifndef _WIN32
#  include <csignal>
#endif

// ── Platform shims ────────────────────────────────────────────────────────────
#ifdef _WIN32
#  include <winsock2.h>
#  include <ws2tcpip.h>
#  include <windows.h>
#  include <io.h>
#  include <fcntl.h>
#  pragma comment(lib, "ws2_32.lib")
   using sock_t   = SOCKET;
   using ssize_t  = int;
#  define BAD_SOCK    INVALID_SOCKET
#  define sock_close  closesocket
// Windows has no MSG_NOSIGNAL; send() doesn't raise signals anyway.
// Broken-pipe is detected via return value only.
#  define MSG_NOSIGNAL 0
#else
#  include <unistd.h>
#  include <sys/socket.h>
#  include <netinet/in.h>
#  include <fcntl.h>
#  include <sys/wait.h>
   using sock_t   = int;
#  define BAD_SOCK    (-1)
#  define sock_close  ::close
#endif

// ── Encoder probing (UNCHANGED) ───────────────────────────────────────────────

static bool ffmpeg_encoder_works(const char *enc)
{
    char cmd[256];
#ifdef _WIN32
    snprintf(cmd, sizeof(cmd),
        "ffmpeg -hide_banner -loglevel error"
        " -f lavfi -i nullsrc=s=160x160:r=1"
        " -vf format=yuv420p"
        " -c:v %s -frames:v 1 -f null - >nul 2>&1", enc);
#else
    snprintf(cmd, sizeof(cmd),
        "ffmpeg -hide_banner -loglevel error"
        " -f lavfi -i nullsrc=s=160x160:r=1"
        " -vf format=yuv420p"
        " -c:v %s -frames:v 1 -f null - 2>/dev/null", enc);
#endif
    int ret = system(cmd);
    LOG_INFO("StreamEncoder: probe %s -> exit %d", enc, ret);
    return ret == 0;
}

static const char *pick_encoder()
{
    for (const char *enc : {"h264_nvenc", "h264_videotoolbox", "h264_qsv"})
        if (ffmpeg_encoder_works(enc)) return enc;
    return "libx264";
}

// ── TCP helpers ───────────────────────────────────────────────────────────────

static sock_t make_server_socket(int port)
{
    sock_t fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd == BAD_SOCK) return BAD_SOCK;

    int yes = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, (char *)&yes, sizeof(yes));

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons((uint16_t)port);

    if (bind(fd, (sockaddr *)&addr, sizeof(addr)) < 0 ||
        listen(fd, 8) < 0)
    {
        sock_close(fd);
        return BAD_SOCK;
    }
    return fd;
}

// Drain a socket until we've seen the blank line that ends HTTP headers
// (\r\n\r\n), or until the client disconnects / kMaxDrain bytes consumed.
// Keeps a 3-byte tail between iterations so the marker can't slip through
// a chunk boundary undetected.
static void drain_http_request(sock_t fd)
{
    static constexpr int kMaxDrain = 8192;
    // tail holds the last 3 bytes of the previous chunk so we can detect
    // \r\n\r\n even when it straddles two recv() calls.
    char tail[3]  = {0, 0, 0};
    int  tail_len = 0;
    int  total    = 0;

    while (total < kMaxDrain)
    {
        char buf[256];
        int n = (int)recv(fd, buf, sizeof(buf), 0);
        if (n <= 0) return;
        total += n;

        // Build window = tail + new bytes, scan for \r\n\r\n.
        char window[3 + 256];
        memcpy(window, tail, tail_len);
        memcpy(window + tail_len, buf, n);
        int wlen = tail_len + n;

        for (int i = 0; i <= wlen - 4; ++i)
            if (window[i]=='\r' && window[i+1]=='\n' &&
                window[i+2]=='\r' && window[i+3]=='\n')
                return;

        // Save last 3 bytes as tail for next iteration.
        tail_len = (wlen >= 3) ? 3 : wlen;
        memcpy(tail, window + wlen - tail_len, tail_len);
    }
}

static void send_http_header(sock_t fd)
{
    const char *hdr =
        "HTTP/1.0 200 OK\r\n"
        "Content-Type: video/mp2t\r\n"
        "Cache-Control: no-cache\r\n"
        "Connection: close\r\n"
        "\r\n";
    // Use a loop in case send() returns a short write (rare on loopback,
    // but correct to handle).
    const char *p   = hdr;
    int         rem = (int)strlen(hdr);
    while (rem > 0)
    {
        int sent = (int)send(fd, p, rem, 0);
        if (sent <= 0) return;
        p   += sent;
        rem -= sent;
    }
}

// Send to one client with a short deadline.
// Returns false only on a hard error (EPIPE, reset, timeout) so transient
// kernel-buffer pressure doesn't immediately drop the client.
// kSendTimeoutMs gives the player time to drain its recv buffer without
// holding the broadcast loop indefinitely.
static constexpr int kSendTimeoutMs = 200;

static bool try_send(sock_t fd, const uint8_t *data, int len)
{
    const char *p   = (const char *)data;
    int         rem = len;

    while (rem > 0)
    {
        fd_set wfds;
        FD_ZERO(&wfds);
        FD_SET(fd, &wfds);
        timeval tv{}; tv.tv_sec = 0; tv.tv_usec = kSendTimeoutMs * 1000;
#ifdef _WIN32
        int ready = select(0,        nullptr, &wfds, nullptr, &tv);
#else
        int ready = select(fd + 1,   nullptr, &wfds, nullptr, &tv);
#endif
        if (ready <= 0) return false;   // timeout or select error → drop

        int sent = (int)send(fd, p, (size_t)rem, MSG_NOSIGNAL);
        if (sent <= 0) return false;    // connection reset / closed
        p   += sent;
        rem -= sent;
    }
    return true;
}

// ── StreamEncoder::Init ───────────────────────────────────────────────────────

bool StreamEncoder::Init(int port, int width, int height, int fps)
{
    m_width   = width;
    m_height  = height;
    m_running = true;

    // ── Init Winsock once, here, not inside make_server_socket ───────────────
#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
    {
        LOG_ERROR("StreamEncoder: WSAStartup failed");
        return false;
    }
    m_wsa_initialized = true;
#endif

    // ── Build ffmpeg command (ALL options UNCHANGED; only output target) ──────
    const char *encoder  = pick_encoder();
    const char *enc_opts = strcmp(encoder, "libx264") == 0
        ? "-preset ultrafast -tune zerolatency"
        : strcmp(encoder, "h264_nvenc") == 0
            ? "-preset p1 -rc cbr -delay 0 -bf 0 -2pass 0 -surfaces 1"
            : "-preset veryfast";

    LOG_INFO("StreamEncoder: using encoder %s", encoder);

    char vf_buf[64];
    if (strcmp(encoder, "h264_nvenc") == 0)
        vf_buf[0] = '\0';
    else
        snprintf(vf_buf, sizeof(vf_buf), "-vf format=yuv420p");

    char cmd[640];
    snprintf(cmd, sizeof(cmd),
        "ffmpeg -loglevel warning"
        " -f rawvideo -pix_fmt rgba -s %dx%d -r %d -i pipe:0"
        " %s"
        " -c:v %s %s"
        " -g 5"
        " -muxdelay 0 -muxpreload 0"
        " -f mpegts -mpegts_flags resend_headers"
        " pipe:1",          // TS goes to stdout; we read it and fan out
        width, height, fps,
        vf_buf,
        encoder, enc_opts);

    LOG_INFO("StreamEncoder: launching: %s", cmd);

    // ── Launch ffmpeg with one pipe for stdin, one for stdout ─────────────────
#ifdef _WIN32
    SECURITY_ATTRIBUTES sa{ sizeof(sa), nullptr, TRUE };
    HANDLE hStdinR{}, hStdinW{}, hStdoutR{}, hStdoutW{};
    if (!CreatePipe(&hStdinR,  &hStdinW,  &sa, 0) ||
        !CreatePipe(&hStdoutR, &hStdoutW, &sa, 0))
    {
        LOG_ERROR("StreamEncoder: CreatePipe failed (err %lu)", GetLastError());
        return false;
    }
    // Our ends must not be inherited.
    SetHandleInformation(hStdinW,  HANDLE_FLAG_INHERIT, 0);
    SetHandleInformation(hStdoutR, HANDLE_FLAG_INHERIT, 0);

    STARTUPINFOA si{};
    si.cb        = sizeof(si);
    si.dwFlags   = STARTF_USESTDHANDLES;
    si.hStdInput  = hStdinR;
    si.hStdOutput = hStdoutW;
    si.hStdError  = GetStdHandle(STD_ERROR_HANDLE);

    PROCESS_INFORMATION pi{};
    char cmdline[768];
    snprintf(cmdline, sizeof(cmdline), "cmd /c %s", cmd);
    BOOL ok = CreateProcessA(nullptr, cmdline, nullptr, nullptr,
                             /*bInheritHandles=*/TRUE, 0,
                             nullptr, nullptr, &si, &pi);
    // Close child-side handles immediately after spawn regardless of outcome.
    CloseHandle(hStdinR);
    CloseHandle(hStdoutW);
    if (!ok)
    {
        CloseHandle(hStdinW);
        CloseHandle(hStdoutR);
        LOG_ERROR("StreamEncoder: CreateProcess failed (err %lu)", GetLastError());
        return false;
    }
    CloseHandle(pi.hProcess);
    CloseHandle(pi.hThread);

    // Wrap native handles in CRT file descriptors so we can use fwrite/_read.
    int wfd = _open_osfhandle((intptr_t)hStdinW,  _O_WRONLY | _O_BINARY);
    int rfd = _open_osfhandle((intptr_t)hStdoutR, _O_RDONLY | _O_BINARY);
    if (wfd < 0 || rfd < 0)
    {
        LOG_ERROR("StreamEncoder: _open_osfhandle failed");
        return false;
    }
    m_pipe    = _fdopen(wfd, "wb");
    m_read_fd = rfd;

#else   // POSIX ─────────────────────────────────────────────────────────────
    int stdin_fds[2], stdout_fds[2];

    // O_CLOEXEC ensures these fds are not leaked into the child after exec.
    // (The child gets them via dup2 before exec, and closes the originals.)
    if (pipe2(stdin_fds,  O_CLOEXEC) ||
        pipe2(stdout_fds, O_CLOEXEC))
    {
        LOG_ERROR("StreamEncoder: pipe2() failed");
        return false;
    }

    pid_t pid = fork();
    if (pid < 0)
    {
        LOG_ERROR("StreamEncoder: fork() failed");
        close(stdin_fds[0]);  close(stdin_fds[1]);
        close(stdout_fds[0]); close(stdout_fds[1]);
        return false;
    }

    if (pid == 0)
    {
        // Child: dup2 drops CLOEXEC on the new fds, then exec.
        dup2(stdin_fds[0],  STDIN_FILENO);
        dup2(stdout_fds[1], STDOUT_FILENO);
        // All other fds (NT sockets, raylib handles, etc.) have O_CLOEXEC
        // and will be closed automatically by execl.
        execl("/bin/sh", "sh", "-c", cmd, nullptr);
        _exit(127);
    }

    // Parent: close child-side ends.
    close(stdin_fds[0]);
    close(stdout_fds[1]);

    m_pipe    = fdopen(stdin_fds[1], "wb");
    m_read_fd = stdout_fds[0];
    m_child_pid = pid;
#endif

    if (!m_pipe)
    {
        LOG_ERROR("StreamEncoder: fdopen on write pipe failed");
        return false;
    }

    // ── Bind the HTTP server ──────────────────────────────────────────────────
    m_server_fd = make_server_socket(port);
    if (m_server_fd == BAD_SOCK)
    {
        LOG_ERROR("StreamEncoder: could not bind on port %d", port);
        return false;
    }

    // ── Reader thread: ffmpeg stdout → fan out to all HTTP clients ────────────
    m_read_thread = std::thread([this]()
    {
        static constexpr int kBuf = 65536;
        std::vector<uint8_t> buf(kBuf);

        while (m_running)
        {
#ifdef _WIN32
            int n = _read(m_read_fd, buf.data(), kBuf);
#else
            int n = (int)read(m_read_fd, buf.data(), kBuf);
#endif
            if (n <= 0) break;

            std::lock_guard<std::mutex> lk(m_clients_mtx);
            m_clients.erase(
                std::remove_if(m_clients.begin(), m_clients.end(),
                    [&](sock_t fd)
                    {
                        bool dead = !try_send(fd, buf.data(), n);
                        if (dead)
                        {
                            LOG_INFO("StreamEncoder: client %llu dropped",
                                     (unsigned long long)fd);
                            sock_close(fd);
                        }
                        return dead;
                    }),
                m_clients.end());
        }
        LOG_INFO("StreamEncoder: reader thread exiting");
    });

    // ── Accept thread: handshake new HTTP clients ─────────────────────────────
    m_accept_thread = std::thread([this]()
    {
        while (m_running)
        {
            sock_t client = accept(m_server_fd, nullptr, nullptr);
            if (client == BAD_SOCK) break;

            drain_http_request(client);
            send_http_header(client);

            {
                std::lock_guard<std::mutex> lk(m_clients_mtx);
                m_clients.push_back(client);
            }
            LOG_INFO("StreamEncoder: client %llu connected",
                     (unsigned long long)client);
        }
        LOG_INFO("StreamEncoder: accept thread exiting");
    });

    LOG_INFO("StreamEncoder: hosting %dx%d @ %d fps -> http://0.0.0.0:%d",
             width, height, fps, port);
    return true;
}

// ── StreamEncoder::Shutdown ───────────────────────────────────────────────────

void StreamEncoder::Shutdown()
{
    if (!m_running) return;
    m_running = false;

    // Closing the write pipe sends ffmpeg EOF → ffmpeg flushes and exits →
    // its stdout closes → read() in the reader thread returns 0 → thread exits.
    if (m_pipe) { fflush(m_pipe); fclose(m_pipe); m_pipe = nullptr; }

    // shutdown(SHUT_RDWR) is defined to unblock any thread blocked in accept()
    // on this fd.  Plain close() from another thread is POSIX UB and in
    // practice often doesn't wake accept() at all (hence the 30s hang).
    if (m_server_fd != BAD_SOCK)
    {
#ifdef _WIN32
        ::shutdown(m_server_fd, SD_BOTH);
#else
        ::shutdown(m_server_fd, SHUT_RDWR);
#endif
        sock_close(m_server_fd);
        m_server_fd = BAD_SOCK;
    }

    if (m_read_thread.joinable())   m_read_thread.join();
    if (m_accept_thread.joinable()) m_accept_thread.join();

    // Close remaining connected clients.
    {
        std::lock_guard<std::mutex> lk(m_clients_mtx);
        for (sock_t fd : m_clients) sock_close(fd);
        m_clients.clear();
    }

#ifdef _WIN32
    if (m_read_fd >= 0) { _close(m_read_fd); m_read_fd = -1; }
    if (m_wsa_initialized) { WSACleanup(); m_wsa_initialized = false; }
#else
    if (m_read_fd >= 0) { ::close(m_read_fd); m_read_fd = -1; }
    if (m_child_pid > 0)
    {
        // Give ffmpeg up to 2s to flush and exit on its own after we closed
        // its stdin.  If it hasn't gone by then, kill it.
        for (int i = 0; i < 20; ++i)
        {
            int status;
            if (waitpid(m_child_pid, &status, WNOHANG) > 0)
            {
                m_child_pid = -1;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (m_child_pid > 0)
        {
            kill(m_child_pid, SIGTERM);
            waitpid(m_child_pid, nullptr, 0);
            m_child_pid = -1;
        }
    }
#endif

    LOG_INFO("StreamEncoder: stopped");
}

// ── StreamEncoder::PushFrame (UNCHANGED) ─────────────────────────────────────

void StreamEncoder::PushFrame(const void *rgba_data, int width, int height)
{
    if (!m_pipe) return;
    size_t bytes   = (size_t)width * height * 4;
    size_t written = fwrite(rgba_data, 1, bytes, m_pipe);
    if (written != bytes)
    {
        LOG_WARN("StreamEncoder: short write (%zu / %zu). ffmpeg may have died",
                 written, bytes);
        fclose(m_pipe);
        m_pipe = nullptr;
    }
}