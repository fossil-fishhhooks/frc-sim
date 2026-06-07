#pragma once
#include <string>
#include <cstdio>

// StreamEncoder — pipes raw RGBA frames into an ffmpeg process that encodes
// H.264 and muxes to MPEG-TS over UDP.
//
// Usage:
//   StreamEncoder enc;
//   enc.Init("127.0.0.1", 5000, 1280, 720, 30);
//   // each frame, after EndDrawing():
//   Image img = LoadImageFromScreen();
//   enc.PushFrame(img.data, img.width, img.height);
//   UnloadImage(img);
//   enc.Shutdown();

class StreamEncoder
{
public:
    // host: destination IP, port: UDP port, fps should match target_fps
    bool Init(const std::string &host, int port, int width, int height, int fps);
    void Shutdown();

    // Push one RGBA frame. data must be width*height*4 bytes.
    void PushFrame(const void *rgba_data, int width, int height);

    bool IsRunning() const { return m_pipe != nullptr; }

private:
    FILE       *m_pipe  = nullptr;
    int         m_width  = 0;
    int         m_height = 0;
};
