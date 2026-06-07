#include "render/StreamEncoder.h"
#include "io/EasyLog.h"
#include <cstring>

bool StreamEncoder::Init(const std::string &host, int port,
                         int width, int height, int fps)
{
    m_width  = width;
    m_height = height;

    // Build ffmpeg command:
    //   -f rawvideo -pix_fmt rgba      — input is raw RGBA from raylib readback
    //   -c:v libx264 -preset ultrafast — low-latency encode
    //   -tune zerolatency              — minimize buffering
    //   -pix_fmt yuv420p               — required for most players
    //   -f mpegts udp://host:port      — MPEG-TS over UDP, no connection needed
    //
    // UDP is fire-and-forget: viewers can connect/disconnect freely.
    // Increase buffer_size if you see drops on a LAN with many clients.
    char cmd[512];
    snprintf(cmd, sizeof(cmd),
        "ffmpeg -loglevel warning"
        " -f rawvideo -pix_fmt rgba -s %dx%d -r %d -i pipe:0"
        " -c:v libx264 -preset ultrafast -tune zerolatency"
        " -pix_fmt yuv420p -g %d"
        " -f mpegts udp://%s:%d?pkt_size=1316"
        " 2>&1",
        width, height, fps,
        fps,           // keyframe every 1 second
        host.c_str(), port);

    LOG_INFO("StreamEncoder: launching: %s", cmd);
    m_pipe = popen(cmd, "w");
    if (!m_pipe)
    {
        LOG_ERROR("StreamEncoder: popen failed! Check is ffmpeg installed?");
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
        pclose(m_pipe);
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
        LOG_WARN("StreamEncoder: short write (%zu / %zu). ffmpeg may have died", written, bytes);
        pclose(m_pipe);
        m_pipe = nullptr;
    }
}
