#include "rm_log.h"

#include <string>

int main() {
    utils::RMLOG::instance().Init("stage4.log", "debug", "info", "debug");
    SET_LOG_LEVEL("debug");
    RM_LOG_INFO("program started");
    const std::string camera_name = "training_camera";
    RM_LOG_DEBUG("selected camera: {}", camera_name);
    RM_LOG_WARN("camera exposure is near its limit: {} us", 9500);
    RM_LOG_ERROR("camera frame timeout after {} ms", 1000);
    utils::RMLOG::instance().Close();
    return 0;
}
