#include "log.hpp"

#include <atomic>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <g3log/logmessage.hpp>
#include <g3log/logworker.hpp>
#include <iomanip>
#include <limits>
#include <locale>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <unistd.h>
#include <utils/poly_traj_utils.hpp>

namespace planner_tools
{
namespace
{
struct Counters
{
    std::atomic<std::uint64_t> submitted{0}, written{0}, dropped{0}, errors{0};
    std::atomic<std::size_t> pending{0}, pending_bytes{0};
};
Counters counters;

std::int64_t epochNanoseconds() noexcept
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

// 兜底不申请 C++ 动态内存；与日志文件及其异步队列完全独立。
void emergency(const char *detail) noexcept
{
    static std::atomic<std::int64_t> next{0};
    const auto now = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    auto expected = next.load();
    if (now < expected || !next.compare_exchange_strong(expected, now + 5))
        return;
    const char prefix[] = "planner log: ";
    (void)::write(STDERR_FILENO, prefix, sizeof(prefix) - 1);
    (void)::write(STDERR_FILENO, detail, std::min<std::size_t>(std::strlen(detail), 256));
    (void)::write(STDERR_FILENO, "\n", 1);
}

/// 保证每条事件只占一个物理行；反斜杠先转义，便于无歧义地恢复消息。
std::string singleLine(const std::string &text)
{
    std::string result;
    result.reserve(text.size());
    for (char c : text)
    {
        if (c == '\\') result += "\\\\";
        else if (c == '\n') result += "\\n";
        else if (c == '\r') result += "\\r";
        else result += c;
    }
    return result;
}

LEVELS g3Level(LogLevel level)
{
    switch (level)
    {
    case LogLevel::Debug: return G3LOG_DEBUG;
    case LogLevel::Warning: return WARNING;
    case LogLevel::Error: return {800, "ERROR"};
    default: return INFO;
    }
}

/// 所有文件操作只在 sink 线程执行（首次创建除外）。
class FileSink
{
public:
    FileSink(std::string path, const LogConfig &config, const std::string &session)
        : path_(std::move(path)), config_(config)
    {
        std::ostringstream header;
        header << "# planner_log_version=2\n# session_id=" << session
               << " software_version=" << std::quoted(singleLine(config.software_version))
               << " system_version=" << std::quoted(singleLine(config.system_version)) << '\n'
               << "# clock=unix_nanoseconds timezone=UTC units=SI numeric_precision=max_digits10\n"
               << "# escaping=backslash_newline_carriage_return\n"
               << "# trajectory_version=1 coefficient_order=descending_local_seconds"
                  " coefficient_layout=3x6_row_major axis_order=x,y,z\n";
        header_ = header.str();
        open();
    }

    void write(g3::LogMessage message, std::int64_t timestamp, std::uint64_t sequence,
        const std::string &tag, LogChannel channel, std::size_t bytes) noexcept
    {
        try
        {
            std::ostringstream line;
            line.imbue(std::locale::classic());
            // 两种时间表示来自同一个采集时间，明确采用 UTC。
            const std::time_t seconds = timestamp / 1000000000;
            std::tm utc{};
            gmtime_r(&seconds, &utc);
            line << "epoch_ns=" << timestamp << " seq=" << sequence
                 << " time=" << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ")
                 << " level=" << message.level()
                 << " channel=" << (channel == LogChannel::Replay ? "replay" : "diagnostic")
                 << " tag=" << std::quoted(singleLine(tag)) << " pid=" << getpid()
                 << " tid=" << message.threadID()
                 << " source=" << std::quoted(singleLine(message.file()))
                 << " line=" << message.line()
                 << " function=" << std::quoted(singleLine(message.function()))
                 << " | " << singleLine(message.message()) << '\n';
            const auto record = line.str();
            if (file_bytes_ > header_.size() && file_bytes_ + record.size() > config_.max_file_bytes)
                rotate();
            file_ << record;
            if (!file_)
                throw std::runtime_error("Failed to write planner log");
            file_bytes_ += record.size();
            if (config_.console)
                (void)::write(STDERR_FILENO, record.data(), record.size());
            const auto now = std::chrono::steady_clock::now();
            if (++unflushed_ >= config_.flush_every || message.level() == "ERROR" ||
                now - last_flush_ >= std::chrono::milliseconds(250))
                flush();
            ++counters.written;
        }
        catch (const std::exception &error)
        {
            ++counters.dropped;
            reportLogFailure(error.what());
        }
        catch (...)
        {
            ++counters.dropped;
            reportLogFailure("Unknown asynchronous write failure");
        }
        --counters.pending;
        counters.pending_bytes.fetch_sub(bytes);
    }

    void receive(g3::LogMessageMover) {} // 此 sink 仅通过受控的 write 队列提交

    void flush()
    {
        file_.flush();
        if (!file_)
            throw std::runtime_error("Failed to flush planner log");
        unflushed_ = 0;
        last_flush_ = std::chrono::steady_clock::now();
    }

private:
    void open()
    {
        file_.clear();
        file_.open(path_, std::ios::out | std::ios::trunc);
        if (!file_)
            throw std::runtime_error("Cannot open log file: " + path_);
        file_.imbue(std::locale::classic());
        file_ << header_;
        flush();
        file_bytes_ = header_.size();
    }

    void rotate()
    {
        flush();
        file_.close();
        // 只管理本 session 的文件，绝不删除其他仍在运行进程的日志。
        if (config_.max_files > 1)
        {
            std::filesystem::remove(path_ + "." + std::to_string(config_.max_files - 1));
            for (std::size_t i = config_.max_files - 1; i > 1; --i)
            {
                const auto previous = path_ + "." + std::to_string(i - 1);
                if (std::filesystem::exists(previous))
                    std::filesystem::rename(previous, path_ + "." + std::to_string(i));
            }
            std::filesystem::rename(path_, path_ + ".1");
        }
        open();
    }

    std::string path_, header_;
    LogConfig config_;
    std::ofstream file_;
    std::size_t file_bytes_ = 0, unflushed_ = 0;
    std::chrono::steady_clock::time_point last_flush_{};
};

class Logger
{
public:
    ~Logger()
    {
        try
        {
            if (sink_) sink_->call(&FileSink::flush).get();
        }
        catch (...) { reportLogFailure("Failed to flush on shutdown"); }
        // 先等待队列结束，再销毁进程计数及其余状态。
        worker_.reset();
    }

    void initialize(const LogConfig &config)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (worker_)
            throw std::logic_error("Planner log already initialized");
        initializeLocked(config);
    }

    bool enabled(LogLevel level, LogChannel channel)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // 尚未初始化时只读默认配置，避免过滤 Debug 也创建文件和线程。
        return channel == LogChannel::Replay ? config_.replay : level >= config_.level;
    }

    void write(LogLevel level, LogChannel channel, const std::string &tag,
        const char *file, int line, const char *function, std::int64_t timestamp,
        const std::string &text)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!worker_) initializeLocked(config_);
        const auto sequence = ++sequence_; // 丢弃也占序号，便于发现记录缺口
        const std::size_t bytes = text.size() + tag.size();
        if (bytes > config_.max_record_bytes ||
            counters.pending.load() >= config_.max_pending_records ||
            bytes > config_.max_pending_bytes - counters.pending_bytes.load())
        {
            ++counters.dropped;
            emergency("Record dropped: queue or record size limit; inspect logHealth()");
            return;
        }
        g3::LogMessage message(file, line, function, g3Level(level));
        message.write() = text;
        ++counters.pending;
        counters.pending_bytes.fetch_add(bytes);
        try
        {
            sink_->call(&FileSink::write, std::move(message), timestamp, sequence, tag, channel, bytes);
            ++counters.submitted;
        }
        catch (...)
        {
            --counters.pending;
            counters.pending_bytes.fetch_sub(bytes);
            throw;
        }
    }

    void flush()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!worker_) initializeLocked(config_);
        sink_->call(&FileSink::flush).get();
    }

    std::string path()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!worker_) initializeLocked(config_);
        return path_;
    }

private:
    void initializeLocked(const LogConfig &config)
    {
        if (config.directory.empty() || config.name.empty() ||
            config.name.find_first_of("/\\\r\n") != std::string::npos ||
            config.level < LogLevel::Debug || config.level > LogLevel::Error ||
            config.max_file_bytes == 0 || config.max_files == 0 ||
            config.max_pending_records == 0 || config.max_pending_bytes == 0 ||
            config.max_record_bytes == 0 || config.max_record_bytes > config.max_pending_bytes ||
            config.flush_every == 0)
            throw std::invalid_argument("Invalid planner log configuration");
        const auto directory = std::filesystem::absolute(config.directory);
        std::filesystem::create_directories(directory);
        const auto session = std::to_string(epochNanoseconds()) + "-" + std::to_string(getpid());
        const auto path = (directory / (config.name + "." + session + ".log")).string();
        auto worker = g3::LogWorker::createLogWorker();
        auto sink = worker->addSink(std::make_unique<FileSink>(path, config, session), &FileSink::receive);
        config_ = config;
        path_ = path;
        worker_ = std::move(worker);
        sink_ = std::move(sink);
    }

    std::mutex mutex_;
    LogConfig config_;
    std::string path_;
    std::uint64_t sequence_ = 0;
    std::unique_ptr<g3::LogWorker> worker_;
    std::unique_ptr<g3::SinkHandle<FileSink>> sink_;
};

Logger &logger()
{
    static Logger value;
    return value;
}
} // namespace

void reportLogFailure(const char *detail) noexcept
{
    ++counters.errors;
    emergency(detail);
}

void initLog(const LogConfig &config)
{
    try { logger().initialize(config); }
    catch (...) { reportLogFailure("Log initialization failed"); throw; }
}

void flushLog()
{
    try { logger().flush(); }
    catch (...) { reportLogFailure("Explicit log flush failed"); throw; }
}

std::string logFile() { return logger().path(); }

LogHealth logHealth() noexcept
{
    return {counters.submitted.load(), counters.written.load(), counters.dropped.load(),
        counters.errors.load(), counters.pending.load()};
}

bool logEnabled(LogLevel level, LogChannel channel) noexcept
{
    try { return logger().enabled(level, channel); }
    catch (...) { reportLogFailure("Cannot check log configuration"); return false; }
}

LogLine::LogLine(LogLevel level, std::string_view tag, const char *file, int line,
    const char *function, LogChannel channel) noexcept
    : level_(level), channel_(channel), file_(file), line_(line), function_(function),
      timestamp_(epochNanoseconds())
{
    try
    {
        tag_ = tag;
        stream_.emplace();
        stream_->imbue(std::locale::classic());
        *stream_ << std::setprecision(std::numeric_limits<double>::max_digits10) << std::boolalpha;
    }
    catch (...) { fail(); }
}

void LogLine::fail() noexcept
{
    stream_.reset();
    ++counters.dropped;
    reportLogFailure("Cannot construct or format log record");
}

LogLine &LogLine::operator<<(std::ostream &(*manipulator)(std::ostream &)) noexcept
{
    if (stream_)
    {
        try
        {
            (*stream_) << manipulator;
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

LogLine::~LogLine() noexcept
{
    if (!stream_) return;
    try
    {
        logger().write(level_, channel_, tag_, file_, line_, function_, timestamp_, stream_->str());
    }
    catch (const std::exception &error)
    {
        ++counters.dropped;
        reportLogFailure(error.what());
    }
    catch (...)
    {
        ++counters.dropped;
        reportLogFailure("Unknown log submission failure");
    }
}

} // namespace planner_tools

namespace poly_traj
{
std::ostream &operator<<(std::ostream &stream, const Trajectory<3> &value)
{
    stream << "trajectory_version=1 pieces=" << value.getPieceNum()
           << " duration=" << value.getTotalDuration()
           << " coefficient_order=descending_local_seconds";
    int index = 0;
    for (const auto &piece : value)
    {
        stream << " piece" << index << "_time=" << piece.getDuration() << " piece" << index
               << "_coeff=[";
        const auto &coefficients = piece.getCoeffMat();
        for (int row = 0; row < 3; ++row)
            for (int col = 0; col < 6; ++col)
            {
                if (row != 0 || col != 0) stream << ',';
                stream << coefficients(row, col);
            }
        stream << ']';
        ++index;
    }
    return stream;
}
} // namespace poly_traj
