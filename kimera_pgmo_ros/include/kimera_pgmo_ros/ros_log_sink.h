#pragma once

#include <ros/ros.h>

#include "kimera_pgmo/utils/logging.h"

namespace kimera_pgmo {

struct RosLogSink : logging::LogSink {
  RosLogSink() = default;
  virtual ~RosLogSink() = default;

  void dispatch(const logging::LogEntry& entry) const override {
    std::stringstream ss;
    ss << entry.prefix() << entry.message();
    switch (entry.level) {
      case logging::Level::WARNING:
        ROS_WARN_STREAM(ss.str());
        break;
      case logging::Level::ERROR:
        ROS_ERROR_STREAM(ss.str());
        break;
      case logging::Level::FATAL:
        ROS_FATAL_STREAM(ss.str());
        break;
      case logging::Level::INFO:
        ROS_INFO_STREAM(ss.str());
        break;
      default:
      case logging::Level::DEBUG:
        ROS_DEBUG_STREAM(ss.str());
        break;
    }
  }
};

}  // namespace kimera_pgmo
