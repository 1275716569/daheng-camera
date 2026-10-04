#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"

#include <memory>

int main() {
    auto console = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    auto file = std::make_shared<spdlog::sinks::basic_file_sink_mt>("stage2.log", true);
    spdlog::logger logger("training", {console, file});
    logger.set_level(spdlog::level::debug);
    logger.set_pattern("[%l] %v");
    logger.info("program started");
    logger.debug("frame id = {}", 1);
    logger.warn("this message is written to both sinks");
    logger.error("camera frame timeout");
    return 0;
}
