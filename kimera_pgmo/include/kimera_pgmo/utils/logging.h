#pragma once
#include <map>
#include <memory>
#include <sstream>
#include <string>

namespace logging {

enum class Level : int { DEBUG = 0, INFO = 1, WARNING = 2, ERROR = 3, FATAL = 4 };

class LogEntry;

struct LogSink {
  using Ptr = std::shared_ptr<LogSink>;
  virtual ~LogSink() = default;
  virtual void dispatch(const LogEntry& entry) const = 0;
};

class Logger {
 public:
  ~Logger() = default;

  static Logger& instance();
  static void addSink(const std::string& name, const LogSink::Ptr& sink);
  static void dispatchLogEntry(const LogEntry& entry);

 private:
  Logger();

  inline static std::unique_ptr<Logger> s_instance_;
  std::map<std::string, LogSink::Ptr> sinks_;
};

class LogEntry {
 public:
  LogEntry(Level level, const std::string& filename, int lineno);

  ~LogEntry();

  Level level;
  std::string filename;
  int lineno;

  std::string prefix() const;
  std::string message() const;

  template <typename T>
  LogEntry& operator<<(const T& rhs) {
    ss_ << rhs;
    return *this;
  }

 private:
  std::stringstream ss_;
};

struct CoutSink : logging::LogSink {
  CoutSink(Level level = Level::INFO);
  virtual ~CoutSink() = default;

  void dispatch(const logging::LogEntry& entry) const override;

  Level level;
};

}  // namespace logging

#define SPARK_LOG(level) logging::LogEntry(logging::Level::level, __FILE__, __LINE__)
