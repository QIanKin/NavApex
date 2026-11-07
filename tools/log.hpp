#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <ostream>
#include <sstream>
#include <string>
#include <string_view>

namespace navapex::poly_traj
{
template <int Dim> class Trajectory;
std::ostream &operator<<(std::ostream &stream, const Trajectory<3> &value);
}

namespace navapex::planner_tools
{
enum class LogLevel { Debug, Info, Warning, Error };
enum class LogChannel { Diagnostic, Replay };

/// 在首次记录前配置一次；轮转保留数量包含当前文件，作用于本次进程会话。
struct LogConfig
{
    std::string directory = "log";
    std::string name = "navapex"; // 文件名：<name>.YYYYMMDD-HHMMSS.log
    std::string software_version = "0.1.0";
    std::string system_version = "unspecified";
    LogLevel level = LogLevel::Info;
    bool console = false;
    bool replay = true; // 独立于诊断等级，保持默认记录轨迹的行为
    std::size_t max_file_bytes = 16 * 1024 * 1024;
    std::size_t max_files = 5;
    std::size_t max_pending_records = 1024;
    std::size_t max_pending_bytes = 16 * 1024 * 1024;
    std::size_t max_record_bytes = 4 * 1024 * 1024;
    std::size_t flush_every = 32; // Error、显式 flush 和正常退出仍立即刷新
};

/// 只读健康快照；计数单调累计。过滤掉的记录不算丢弃。
struct LogHealth
{
    std::uint64_t submitted = 0;
    std::uint64_t written = 0;
    std::uint64_t dropped = 0;
    std::uint64_t errors = 0;
    std::size_t pending = 0;
};

/// 显式管理操作通过异常报告失败；普通日志调用尽力记录，不传播日志自身异常。
void initLog(const LogConfig &config = {});
void flushLog();
std::string logFile();
LogHealth logHealth() noexcept;
bool logEnabled(LogLevel level, LogChannel channel = LogChannel::Diagnostic) noexcept;

/// 报告日志基础设施故障；兜底 stderr 每五秒最多一次，不递归使用日志。
void reportLogFailure(const char *detail) noexcept;

/// 本地时间 HH:MM:SS mmm(pid)级别/标签: 消息；每条自动换行，消息内换行转义。
/// 自身构造、格式化、提交异常全部隔离；日志实参须为无副作用的数据表达式。
class LogLine
{
public:
    LogLine(LogLevel level, std::string_view tag, const char *file, int line,
        const char *function, LogChannel channel = LogChannel::Diagnostic) noexcept;
    ~LogLine() noexcept;
    LogLine(const LogLine &) = delete;
    LogLine &operator=(const LogLine &) = delete;

    template <typename T> LogLine &operator<<(const T &value) noexcept
    {
        if (stream_)
        {
            try
            {
                *stream_ << value;
                if (!*stream_)
                    fail();
            }
            catch (...)
            {
                fail();
            }
        }
        return *this;
    }
    LogLine &operator<<(std::ostream &(*manipulator)(std::ostream &)) noexcept;

private:
    void fail() noexcept;
    LogLevel level_;
    LogChannel channel_;
    std::string tag_;
    const char *file_;
    int line_;
    const char *function_;
    std::int64_t timestamp_;
    std::optional<std::ostringstream> stream_;
};
} // namespace navapex::planner_tools

// for 保证作为单条语句使用时没有悬空 else；过滤后不求值日志实参。
#define PLANNER_LOG_AT(level, tag)                                                                \
    for (bool planner_log_once_ = ::navapex::planner_tools::logEnabled(level); planner_log_once_;            \
         planner_log_once_ = false)                                                              \
        ::navapex::planner_tools::LogLine(level, tag, __FILE__, __LINE__, __func__)
#define LOGD(tag) PLANNER_LOG_AT(::navapex::planner_tools::LogLevel::Debug, tag)
#define LOGI(tag) PLANNER_LOG_AT(::navapex::planner_tools::LogLevel::Info, tag)
#define LOGW(tag) PLANNER_LOG_AT(::navapex::planner_tools::LogLevel::Warning, tag)
#define LOGE(tag) PLANNER_LOG_AT(::navapex::planner_tools::LogLevel::Error, tag)
#define LOG_TRAJECTORY(tag)                                                                       \
    for (bool planner_log_once_ = ::navapex::planner_tools::logEnabled(                                     \
             ::navapex::planner_tools::LogLevel::Info, ::navapex::planner_tools::LogChannel::Replay);                \
         planner_log_once_; planner_log_once_ = false)                                            \
        ::navapex::planner_tools::LogLine(::navapex::planner_tools::LogLevel::Info, tag, __FILE__, __LINE__,          \
            __func__, ::navapex::planner_tools::LogChannel::Replay)
