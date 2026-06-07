#include "render/StreamEncoder.h"
#include "io/EasyLog.h"
#include <cstring>
#include <initializer_list>

// ── Platform shims ────────────────────────────────────────────────────────────
#ifdef _WIN32
#  include <cstdio>
// _popen on Windows requires "wb" for binary pipe writes; the mode string
// differs from POSIX popen which uses "w" (always binary on POSIX pipes).
#  define POPEN(cmd)  _popen((cmd), "wb")
#  define PCLOSE(f)   _pclose((f))
#else
#  define POPEN(cmd)  popen((cmd), "w")
#  define PCLOSE(f)   pclose((f))
#endif

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

bool StreamEncoder::Init(const std::string &host, int port,
                         int width, int height, int fps)
{
    m_width  = width;
    m_height = height;

    // Build ffmpeg command:
    //   -f rawvideo -pix_fmt rgba      — input is raw RGBA from raylib readback
    //   -tune zerolatency              — minimize buffering
    //   -pix_fmt yuv420p               — required for most players
    //   -f mpegts udp://host:port      — MPEG-TS over UDP, no connection needed
    //
    // pkt_size=1316 keeps UDP datagrams under typical MTU (2 * 188-byte TS
    // packets). Omitted on Windows because some builds of ffmpeg for Windows
    // misparse URL options and drop the stream entirely.
        const char *encoder = pick_encoder();
        const char *enc_opts = strcmp(encoder, "libx264") == 0
    ? "-preset ultrafast -tune zerolatency"
    : strcmp(encoder, "h264_nvenc") == 0
        ? "-preset p1 -rc cbr -delay 0 -bf 0 -2pass 0 -surfaces 1"
        : "-preset veryfast";

        LOG_INFO("StreamEncoder: using encoder %s", encoder);

    char cmd[512];
    char vf_buf[64];
if (strcmp(encoder, "h264_nvenc") == 0)
    vf_buf[0] = '\0';   // no -vf at all, feed rgba directly
else
    snprintf(vf_buf, sizeof(vf_buf), "-vf format=yuv420p");

snprintf(cmd, sizeof(cmd),
    "ffmpeg -loglevel warning"
    " -f rawvideo -pix_fmt rgba -s %dx%d -r %d -i pipe:0"
    " %s"                          // empty for nvenc, "-vf format=yuv420p" for others
    " -c:v %s %s"
    " -g %d"
    " -f mpegts -mpegts_flags resend_headers"
#ifdef _WIN32
    " udp://%s:%d",
#else
    " udp://%s:%d?pkt_size=1316",
#endif
    width, height, fps,
    vf_buf,
    encoder, enc_opts,
    fps,
    host.c_str(), port);


    LOG_INFO("StreamEncoder: launching: %s", cmd);
    m_pipe = POPEN(cmd);
    if (!m_pipe)
    {
        LOG_ERROR("StreamEncoder: popen failed. check is ffmpeg in PATH?");
        return false;
    }

    LOG_INFO("StreamEncoder: streaming %dx%d @ %d fps -> udp://%s:%d",
             width, height, fps, host.c_str(), port);
    return true;
}

void StreamEncoder::Shutdown()
{
    if (m_pipe)
    {
        fflush(m_pipe);   // flush any buffered partial frame before closing
        PCLOSE(m_pipe);
        m_pipe = nullptr;
        LOG_INFO("StreamEncoder: stopped");
    }
}

void StreamEncoder::PushFrame(const void *rgba_data, int width, int height)
{
    if (!m_pipe) return;
    size_t bytes = (size_t)width * height * 4;
    size_t written = fwrite(rgba_data, 1, bytes, m_pipe);
    if (written != bytes)
    {
        LOG_WARN("StreamEncoder: short write (%zu / %zu). ffmpeg may have died",
                 written, bytes);
        PCLOSE(m_pipe);
        m_pipe = nullptr;
    }
}