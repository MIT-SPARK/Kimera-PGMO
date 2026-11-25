#include "kimera_pgmo/utils/logging.h"

#include <spdlog/sinks/stdout_color_sinks.h>

namespace kimera_pgmo {

std::shared_ptr<spdlog::logger> getLogger() {
  static const auto logger = [] {
    if (auto existing = spdlog::get("kimera_pgmo")) {
      return existing;
    }

    auto result = spdlog::stderr_color_mt("kimera_pgmo");
    result->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] [%s:%#] %v");
    return result;
  }();
  return logger;
}

}  // namespace kimera_pgmo
