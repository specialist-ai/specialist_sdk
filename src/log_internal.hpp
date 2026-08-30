// Private to the library: the public header has only the handler setters.

#pragma once

#include <string>

#include "specialist/log.hpp"

namespace specialist {

/// Calls the current handler. Catches anything the handler throws.
void log_message(LogLevel level, const std::string& message) noexcept;

}  // namespace specialist
