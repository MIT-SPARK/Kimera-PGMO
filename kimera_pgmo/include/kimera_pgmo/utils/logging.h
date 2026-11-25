#pragma once

#include <spdlog/fmt/ostr.h>
#include <spdlog/fmt/ranges.h>
#include <spdlog/fmt/std.h>
#include <spdlog/spdlog.h>

#ifdef SPDLOG_USE_STD_FORMAT
#error "Logging requires an fmt-based spdlog build"
#endif

#include <memory>
#include <sstream>

namespace kimera_pgmo {

// Applications configure this named logger's sinks and severity directly.
std::shared_ptr<spdlog::logger> getLogger();

}  // namespace kimera_pgmo

// Keep native spdlog source locations and compile-time severity filtering.
#define PGMO_LOG(LEVEL, ...) \
  SPDLOG_LOGGER_##LEVEL(::kimera_pgmo::getLogger(), __VA_ARGS__)
