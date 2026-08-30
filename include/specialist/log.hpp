// Diagnostics go to one process-wide handler. The default handler writes warnings and errors to
// stderr. The handler runs on the I/O thread and on the threads that call lifecycle methods, with
// no lock held, so it must be thread safe and fast. It must not call a lifecycle method. The
// library drops an exception that escapes the handler.

#pragma once

#include <functional>
#include <string>

namespace specialist {

enum class LogLevel { kDebug, kInfo, kWarn, kError };

using LogHandler = std::function<void(LogLevel, const std::string&)>;

/// Installs `handler` for every later message. An empty handler drops all messages.
void set_log_handler(LogHandler handler);
void reset_log_handler();  // installs the default handler again

}  // namespace specialist
