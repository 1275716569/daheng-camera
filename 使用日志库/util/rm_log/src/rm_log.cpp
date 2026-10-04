#include "rm_log.h"

#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/async.h"

namespace utils {

void RMLOG::Init(const std::string &path, const std::string &console_level,
                 const std::string &file_level, const std::string &log_level) {
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::from_str(console_level));

    auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(path.c_str(), 1024 * 1024 * 5, 3);
    file_sink->set_level(spdlog::level::from_str(file_level));

    spdlog::sinks_init_list sink_list = {file_sink, console_sink};

    spdlog::init_thread_pool(4096, 1);
    logger_sptr_ = std::make_shared<spdlog::async_logger>("multi_sink", sink_list.begin(), sink_list.end(),
                                                          spdlog::thread_pool(), spdlog::async_overflow_policy::block);
//    logger_sptr_ = std::make_shared<spdlog::logger>("multi_sink", sink_list.begin(), sink_list.end());
    logger_sptr_->set_level(spdlog::level::from_str(log_level));
    logger_sptr_->flush_on(spdlog::level::warn); // 当遇到 warn 消息级别以上的立刻刷新到日志，以便不正常退出时查看日志
    //设置格式
    //参见文档 https://github.com/gabime/spdlog/wiki/3.-Custom-formatting
    //[%Y-%m-%d %H:%M:%S.%e] 时间
    //[%l] 日志级别
    //[%t] 线程
    //[%s] 文件
    //[%#] 行号
    //[%!] 函数
    //[%v] 实际文本
    logger_sptr_->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [thread %t] [%s %!:%#] %v");

}

void RMLOG::SetLevel(const std::string &log_level) {
    auto level = spdlog::level::from_str(log_level);
    if (level == spdlog::level::n_levels) {
        RM_LOG_WARN("Given invalid log level {}", log_level);
    } else {
        logger_sptr_->set_level(level);
    }
}

}
