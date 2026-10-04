#include <cstdio>

enum LogLevel {
    LOG_INFO,
    LOG_WARN,
    LOG_ERROR
};

void write_log(LogLevel level, const char *message) {
    const char *name = "INFO";
    if (level == LOG_WARN) {
        name = "WARN";
    } else if (level == LOG_ERROR) {
        name = "ERROR";
    }
    std::printf("[%s] %s\n", name, message);
}

int main() {
    write_log(LOG_INFO, "program started");
    write_log(LOG_WARN, "camera exposure is near its limit");
    write_log(LOG_ERROR, "camera frame timeout");
    return 0;
}
