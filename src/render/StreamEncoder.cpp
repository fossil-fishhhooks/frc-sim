#include "render/StreamEncoder.h"
#include "io/EasyLog.h"

bool StreamEncoder::Init(int port, int width, int height, int fps) {
    LOG_INFO("StreamEncoder: dummy — streaming disabled");
    return true;
}

void StreamEncoder::PushFrame(const void* rgba_data, int width, int height) {
    // dummy — no-op
}

void StreamEncoder::Shutdown() {
    LOG_INFO("StreamEncoder: dummy shutdown");
}
