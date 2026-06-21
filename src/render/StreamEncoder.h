#pragma once

class StreamEncoder {
public:
    bool Init(int port, int width, int height, int fps);
    void PushFrame(const void* rgba_data, int width, int height);
    void Shutdown();
    bool IsRunning() const { return false; }
};
