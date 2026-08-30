#include "specialist/log.hpp"

#include <cstdio>
#include <memory>
#include <mutex>
#include <utility>

#include "log_internal.hpp"

namespace specialist {

namespace {

void default_handler(LogLevel level, const std::string& message) {
  if (level == LogLevel::kDebug || level == LogLevel::kInfo) {
    return;
  }
  std::fprintf(stderr, "[specialist] [%s] %s\n", level == LogLevel::kWarn ? "WARN" : "ERROR",
               message.c_str());
}

struct Registry {
  std::mutex mutex;  // guards the pointer only; a call runs on its own copy
  std::shared_ptr<const LogHandler> handler;
};

// Never destroyed: the I/O thread of a Gripper with static storage can log during exit.
Registry& registry() {
  static Registry* const instance = [] {
    auto* created = new Registry;
    created->handler = std::make_shared<const LogHandler>(default_handler);
    return created;
  }();
  return *instance;
}

}  // namespace

void set_log_handler(LogHandler handler) {
  std::shared_ptr<const LogHandler> next;
  if (handler) {
    next = std::make_shared<const LogHandler>(std::move(handler));
  }
  {
    const std::lock_guard<std::mutex> lock(registry().mutex);
    registry().handler.swap(next);
  }
  // `next` now holds the old handler. It is destroyed here, outside the lock,
  // unless a running call still holds it.
}

void reset_log_handler() { set_log_handler(default_handler); }

void log_message(LogLevel level, const std::string& message) noexcept {
  try {
    std::shared_ptr<const LogHandler> handler;
    {
      const std::lock_guard<std::mutex> lock(registry().mutex);
      handler = registry().handler;
    }
    if (handler) {
      (*handler)(level, message);
    }
  } catch (...) {
    // A throwing handler must not stop the I/O thread with the torque on.
  }
}

}  // namespace specialist
