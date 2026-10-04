#ifndef EVO_RMCV_RM_LOG_H_
#define EVO_RMCV_RM_LOG_H_

#include "spdlog/spdlog.h"

namespace utils {

// 日志相关操作的宏封装，以便不封装spdlog现有的api接口，直接调用
#define INIT_LOG(path, console_level, file_level, log_level)      utils::RMLOG::instance().Init(path, console_level, file_level, log_level)
#define SET_LOG_LEVEL(log_level) utils::RMLOG::instance().SetLevel(log_level)
#define RM_LOG_BASE(logger, level, ...) (logger)->log(spdlog::source_loc{__FILE__, __LINE__, __func__}, level, __VA_ARGS__)
#define RM_LOG_TRACE(...)     RM_LOG_BASE(utils::RMLOG::instance().getLogger(), spdlog::level::trace, __VA_ARGS__)
#define RM_LOG_DEBUG(...)     RM_LOG_BASE(utils::RMLOG::instance().getLogger(), spdlog::level::debug, __VA_ARGS__)
#define RM_LOG_INFO(...)      RM_LOG_BASE(utils::RMLOG::instance().getLogger(), spdlog::level::info, __VA_ARGS__)
#define RM_LOG_WARN(...)      RM_LOG_BASE(utils::RMLOG::instance().getLogger(), spdlog::level::warn, __VA_ARGS__)
#define RM_LOG_ERROR(...)     RM_LOG_BASE(utils::RMLOG::instance().getLogger(), spdlog::level::err, __VA_ARGS__)
#define RM_LOG_CRITICAL(...)  RM_LOG_BASE(utils::RMLOG::instance().getLogger(), spdlog::level::critical, __VA_ARGS__)

class RMLOG {
public:
    static RMLOG &instance() {
        static RMLOG rm_log;
        return rm_log;
    }

    std::shared_ptr<spdlog::logger> getLogger() {
        return logger_sptr_;
    }

    void Close() {
        spdlog::shutdown();
        spdlog::drop_all();
    }

    void Init(const std::string &path, const std::string &console_level,
              const std::string &file_level, const std::string &log_level);

    void SetLevel(const std::string &log_level);

private:
    RMLOG() = default;

    std::shared_ptr<spdlog::logger> logger_sptr_;

};

}

#endif //EVO_RMCV_RM_LOG_H_
