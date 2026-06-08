#include "render/StreamEncoder.h"
#include "io/EasyLog.h"
#include <cstring>
#include <initializer_list>
#include <thread>
#include <mutex>
#include <vector>
#include <atomic>
#include <algorithm>

// ── Platform shims ────────────────────────────────────────────────────────────
#ifdef _WIN32
#  include <winsock2.h>
#  include <ws2tcpip.h>
#  include <windows.h>
#  include <io.h>
#  include <fcntl.h>
#  pragma comment(lib, "ws2_32.lib")
   using ssize_t = int;
#  define sock_close  closesocket
#  define SOCK_T      SOCKET
#  define BAD_SOCK    INVALID_SOCKET
#  define MSG_NOSIGNAL 0
#else
#  include <unistd.h>
#  include <sys/socket.h>
#  include <netinet/in.h>
#  define sock_close  ::close
#  define SOCK_T      int
#  define BAD_SOCK    (-1)
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

// ── TCP server helpers ────────────────────────────────────────────────────────

static SOCK_T make_server_socket(int port)
{
#ifdef _WIN32
    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);
#endif
    SOCK_T fd = socket(AF_INET, SOCK_STREAM, 0);
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

static void send_http_header(SOCK_T fd)
{
    const char *hdr =
        "HTTP/1.0 200 OK\r\n"
        "Content-Type: video/mp2t\r\n"
        "Cache-Control: no-cache\r\n"
        "Connection: close\r\n"
        "\r\n";
    send(fd, hdr, (int)strlen(hdr), 0);
}

// ── StreamEncoder::Init ───────────────────────────────────────────────────────

bool StreamEncoder::Init(int port, int width, int height, int fps)
{
    m_width  = width;
    m_height = height;
    m_running = true;

    // ── Build ffmpeg command (options UNCHANGED, output changed to pipe:1) ────
    const char *encoder = pick_encoder();
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
        " pipe:1",                      // write TS to stdout; we fan it out
        width, height, fps,
        vf_buf,
        encoder, enc_opts);

    LOG_INFO("StreamEncoder: launching: %s", cmd);

    // ── Launch ffmpeg with bidirectional pipes ────────────────────────────────
#ifdef _WIN32
    SECURITY_ATTRIBUTES sa{ sizeof(sa), nullptr, TRUE };

    HANDLE hStdinR,  hStdinW;
    HANDLE hStdoutR, hStdoutW;
    if (!CreatePipe(&hStdinR,  &hStdinW,  &sa, 0) ||
        !CreatePipe(&hStdoutR, &hStdoutW, &sa, 0))
    {
        LOG_ERROR("StreamEncoder: CreatePipe failed");
        return false;
    }
    // The handles we keep must NOT be inherited by the child.
    SetHandleInformation(hStdinW,  HANDLE_FLAG_INHERIT, 0);
    SetHandleInformation(hStdoutR, HANDLE_FLAG_INHERIT, 0);

    STARTUPINFOA si{};
    si.cb          = sizeof(si);
    si.dwFlags     = STARTF_USESTDHANDLES;
    si.hStdInput   = hStdinR;
    si.hStdOutput  = hStdoutW;
    si.hStdError   = GetStdHandle(STD_ERROR_HANDLE);

    PROCESS_INFORMATION pi{};
    // cmd is already a full shell command; wrap it so CreateProcess finds ffmpeg
    char cmdline[768];
    snprintf(cmdline, sizeof(cmdline), "cmd /c %s", cmd);
    if (!CreateProcessA(nullptr, cmdline, nullptr, nullptr,
                        TRUE, 0, nullptr, nullptr, &si, &pi))
    {
        LOG_ERROR("StreamEncoder: CreateProcess failed");
        return false;
    }
    CloseHandle(hStdinR);
    CloseHandle(hStdoutW);
    CloseHandle(pi.hProcess);
    CloseHandle(pi.hThread);

    int wfd = _open_osfhandle((intptr_t)hStdinW,  _O_WRONLY | _O_BINARY);
    int rfd = _open_osfhandle((intptr_t)hStdoutR, _O_RDONLY | _O_BINARY);
    m_pipe    = _fdopen(wfd, "wb");
    m_read_fd = rfd;

#else   // POSIX ────────────────────────────────────────────────────────────────
    int stdin_fds[2], stdout_fds[2];
    if (pipe(stdin_fds) || pipe(stdout_fds))
    {
        LOG_ERROR("StreamEncoder: pipe() failed");
        return false;
    }

    pid_t pid = fork();
    if (pid < 0)
    {
        LOG_ERROR("StreamEncoder: fork() failed");
        return false;
    }

    if (pid == 0)
    {
        // Child: wire stdin/stdout and exec ffmpeg via shell.
        dup2(stdin_fds[0],  STDIN_FILENO);
        dup2(stdout_fds[1], STDOUT_FILENO);
        close(stdin_fds[0]);  close(stdin_fds[1]);
        close(stdout_fds[0]); close(stdout_fds[1]);
        execl("/bin/sh", "sh", "-c", cmd, nullptr);
        _exit(127);
    }

    // Parent: keep write end of stdin, read end of stdout.
    close(stdin_fds[0]);
    close(stdout_fds[1]);

    m_pipe    = fdopen(stdin_fds[1], "wb");
    m_read_fd = stdout_fds[0];
#endif

    if (!m_pipe)
    {
        LOG_ERROR("StreamEncoder: fdopen on write pipe failed");
        return false;
    }

    // ── Bind the HTTP server socket ───────────────────────────────────────────
    m_server_fd = (int)make_server_socket(port);
    if (m_server_fd == (int)BAD_SOCK)
    {
        LOG_ERROR("StreamEncoder: could not bind on port %d", port);
        return false;
    }

    // ── Reader thread: drain ffmpeg stdout → broadcast to all clients ─────────
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
            std::vector<SOCK_T> dead;

            for (SOCK_T fd : m_clients)
            {
                ssize_t sent = send(fd, (char *)buf.data(), n, MSG_NOSIGNAL);
                if (sent <= 0) dead.push_back(fd);
            }

            for (SOCK_T fd : dead)
            {
                LOG_INFO("StreamEncoder: client disconnected (fd %d)", (int)fd);
                sock_close(fd);
                m_clients.erase(
                    std::remove(m_clients.begin(), m_clients.end(), fd),
                    m_clients.end());
            }
        }
        LOG_INFO("StreamEncoder: reader thread exiting");
    });

    // ── Accept thread: handshake incoming HTTP clients ────────────────────────
    m_accept_thread = std::thread([this]()
    {
        while (m_running)
        {
            SOCK_T client = accept((SOCK_T)m_server_fd, nullptr, nullptr);
            if (client == BAD_SOCK) break;

            // Drain the HTTP request line (we don't care about the path).
            char trash[1024];
            recv(client, trash, sizeof(trash) - 1, 0);

            send_http_header(client);

            {
                std::lock_guard<std::mutex> lk(m_clients_mtx);
                m_clients.push_back(client);
            }
            LOG_INFO("StreamEncoder: client connected (fd %d)", (int)client);
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
    m_running = false;

    // Closing the write pipe sends EOF to ffmpeg, which flushes and exits,
    // which closes its stdout, which unblocks the reader thread's read().
    if (m_pipe) { fflush(m_pipe); fclose(m_pipe); m_pipe = nullptr; }

    // Closing the server socket unblocks accept().
    if (m_server_fd != (int)BAD_SOCK)
    {
        sock_close((SOCK_T)m_server_fd);
        m_server_fd = (int)BAD_SOCK;
    }

    if (m_read_thread.joinable())   m_read_thread.join();
    if (m_accept_thread.joinable()) m_accept_thread.join();

    // Close any remaining connected clients.
    {
        std::lock_guard<std::mutex> lk(m_clients_mtx);
        for (SOCK_T fd : m_clients) sock_close(fd);
        m_clients.clear();
    }

#ifdef _WIN32
    if (m_read_fd >= 0) { _close(m_read_fd); m_read_fd = -1; }
    WSACleanup();
#else
    if (m_read_fd >= 0) { ::close(m_read_fd); m_read_fd = -1; }
#endif

    LOG_INFO("StreamEncoder: stopped");
}

// ── StreamEncoder::PushFrame (UNCHANGED) ─────────────────────────────────────

void StreamEncoder::PushFrame(const void *rgba_data, int width, int height)
{
    if (!m_pipe) return;
    size_t bytes = (size_t)width * height * 4;
    size_t written = fwrite(rgba_data, 1, bytes, m_pipe);
    if (written != bytes)
    {
        LOG_WARN("StreamEncoder: short write (%zu / %zu). ffmpeg may have died",
                 written, bytes);
        fclose(m_pipe);
        m_pipe = nullptr;
    }
}