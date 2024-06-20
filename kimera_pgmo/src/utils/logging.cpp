#include "kimera_pgmo/utils/logging.h"

#include <filesystem>
#include <iostream>

namespace logging {

Logger& Logger::instance() {
  if (!s_instance_) {
    s_instance_.reset(new Logger());
  }

  return *s_instance_;
}

void Logger::addSink(const std::string& name, const LogSink::Ptr& sink) {
  auto& self = instance();
  self.sinks_[name] = sink;
}

void Logger::dispatchLogEntry(const LogEntry& entry) {
  auto& self = instance();
  for (const auto& name_sink_pair : self.sinks_) {
    name_sink_pair.second->dispatch(entry);
  }
}

Logger::Logger() {}

LogEntry::LogEntry(Level level, const std::string& filename, int lineno)
    : level(level), filename(filename), lineno(lineno) {}

LogEntry::~LogEntry() { Logger::dispatchLogEntry(*this); }

std::string LogEntry::prefix() const {
  std::stringstream ss;
  ss << "[" << std::filesystem::path(filename).filename().string();
  if (lineno > 0) {
    ss << ":" << lineno;
  }
  ss << "] ";
  return ss.str();
}

std::string LogEntry::message() const { return ss_.str(); }

CoutSink::CoutSink(Level level) : level(level) {}

void CoutSink::dispatch(const logging::LogEntry& entry) const {
  if (entry.level < level) {
    // skip ignored entries
    return;
  }

  std::stringstream ss;
  ss << entry.prefix() << entry.message();
  switch (entry.level) {
    case logging::Level::WARNING:
      std::cerr << "[WARNING]" << ss.str() << std::endl;
      break;
    case logging::Level::ERROR:
      std::cerr << "[ERROR]" << ss.str() << std::endl;
      break;
    case logging::Level::FATAL:
      std::cerr << "[FATAL]" << ss.str() << std::endl;
      break;
    case logging::Level::INFO:
      std::cout << "[INFO]" << ss.str() << std::endl;
      break;
    default:
    case logging::Level::DEBUG:
      std::cout << "[DEBUG]" << ss.str() << std::endl;
      break;
  }
}

}  // namespace logging
