#pragma once

#include <kimera_pgmo/utils/logging.h>

#include <rclcpp/rclcpp.hpp>

namespace kimera_pgmo {

struct RosLogSink : logging::LogSink {
  explicit RosLogSink(const rclcpp::Logger& logger) : logger(logger) {}
  virtual ~RosLogSink() = default;

  void dispatch(const logging::LogEntry& entry) const override {
    std::stringstream ss;
    ss << entry.prefix() << entry.message();
    switch (entry.level) {
      case logging::Level::WARNING:
        RCLCPP_WARN_STREAM(logger, ss.str());
        break;
      case logging::Level::ERROR:
        RCLCPP_ERROR_STREAM(logger, ss.str());
        break;
      case logging::Level::FATAL:
        RCLCPP_FATAL_STREAM(logger, ss.str());
        break;
      case logging::Level::INFO:
        RCLCPP_INFO_STREAM(logger, ss.str());
        break;
      default:
      case logging::Level::DEBUG:
        RCLCPP_DEBUG_STREAM(logger, ss.str());
        break;
    }
  }

  rclcpp::Logger logger;
};

}  // namespace kimera_pgmo
