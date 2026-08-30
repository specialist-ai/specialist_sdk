// connect() and activate() raise RuntimeError with last_error() when they fail.
// disconnect() and deactivate() return False when a servo did not acknowledge the release.

#include <nanobind/nanobind.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "specialist/gripper.hpp"
#include "specialist/log.hpp"

namespace nb = nanobind;
using namespace nb::literals;
using Blocking = nb::call_guard<nb::gil_scoped_release>;
using specialist::Gripper, specialist::GripperConfig, specialist::JointConfig,
    specialist::JointState;

namespace {

// ~Gripper joins the I/O thread, which can take the GIL to log. nanobind destroys an object with
// the GIL held, so the destructor must release it, or the join deadlocks with the torque on.
struct PyGripper : Gripper {
  ~PyGripper() {
    if (Py_IsInitialized() == 0) return;
    nb::gil_scoped_release release;
    disconnect();
  }
};

// Binds a method that takes a joint index. The C++ call ignores a bad index, and Python raises.
template <auto Method, typename... Args>
auto indexed() {
  return [](PyGripper& self, long long joint, Args... args) {
    // The call goes through the base type, because GCC warns about strict aliasing otherwise.
    Gripper& gripper = self;
    if (joint < 0 || static_cast<std::size_t>(joint) >= gripper.joint_count()) {
      throw nb::index_error(("joint index " + std::to_string(joint) + " is out of range").c_str());
    }
    return (gripper.*Method)(static_cast<std::size_t>(joint), args...);
  };
}

// Binds a lifecycle method that returns false on failure. Python raises with last_error().
template <auto Method, typename... Args>
auto raising() {
  return [](PyGripper& self, Args... args) {
    Gripper& gripper = self;
    if (!(gripper.*Method)(args...)) throw std::runtime_error(gripper.last_error());
  };
}

// Adds copy support, and a repr() that prints each property in the order of definition.
template <typename T>
void value_type(nb::class_<T>& cls) {
  cls.def(nb::init<>())
      .def(nb::init<const T&>(), "other"_a)
      .def("__copy__", [](const T& self) { return T(self); })
      .def(
          "__deepcopy__", [](const T& self, nb::handle) { return T(self); }, "memo"_a.none())
      .def("__repr__", [](nb::handle self) {
        std::string text = nb::str(self.type().attr("__name__")).c_str() + std::string("(");
        const nb::object members = self.type().attr("__dict__");
        for (nb::handle name : members) {
          if (PyObject_IsInstance(members[name].ptr(), (PyObject*)&PyProperty_Type) != 1) continue;
          text += (text.back() == '(' ? "" : ", ") + std::string(nb::str(name).c_str()) + "=" +
                  nb::repr(self.attr(name)).c_str();
        }
        return text + ")";
      });
}

}  // namespace

NB_MODULE(_core, m) {
  nb::enum_<specialist::LogLevel>(m, "LogLevel")
      .value("DEBUG", specialist::LogLevel::kDebug)
      .value("INFO", specialist::LogLevel::kInfo)
      .value("WARN", specialist::LogLevel::kWarn)
      .value("ERROR", specialist::LogLevel::kError);

  m.def(
      "set_log_handler",
      [](std::optional<nb::callable> handler) {
        if (!handler) return specialist::reset_log_handler();
        specialist::set_log_handler(
            [callback = *handler](specialist::LogLevel level, const std::string& message) {
              if (Py_IsInitialized() == 0) return;
              nb::gil_scoped_acquire gil;
              try {
                callback(level, message);
              } catch (nb::python_error& error) {
                error.restore();
                PyErr_WriteUnraisable(callback.ptr());
              }
            });
      },
      "handler"_a.none(), "Call handler(level, message) for each message. None restores stderr.");
  // A Python handler that a C++ static holds after Py_Finalize crashes the exit.
  nb::module_::import_("atexit").attr("register")(
      nb::cpp_function([]() { specialist::reset_log_handler(); }));

  nb::class_<JointConfig> joint(m, "JointConfig");
  joint.def_rw("name", &JointConfig::name)
      .def_rw("id", &JointConfig::id)
      .def_rw("direction", &JointConfig::direction, "+1 or -1")
      .def_rw("homing_offset", &JointConfig::homing_offset, "rad")
      .def_rw("lower_limit", &JointConfig::lower_limit, "rad, the closed side")
      .def_rw("upper_limit", &JointConfig::upper_limit, "rad, the open stop")
      .def_rw("torque_limit", &JointConfig::torque_limit, "permille of stall");
  value_type(joint);

  nb::class_<GripperConfig> config(m, "GripperConfig");
  config.def_rw("port", &GripperConfig::port)
      .def_rw("baud_rate", &GripperConfig::baud_rate)
      .def_rw("bus_rate_hz", &GripperConfig::bus_rate_hz)
      // A tuple of copies, because `joints[0].x = v` on a list would change a copy silently.
      .def_prop_rw(
          "joints",
          [](const GripperConfig& self) {
            nb::list items;
            for (const auto& j : self.joints) items.append(nb::cast(j));
            return nb::tuple(items);
          },
          [](GripperConfig& self, std::vector<JointConfig> joints) { self.joints = joints; },
          nb::for_getter(nb::sig("def joints(self) -> tuple[JointConfig, ...]")),
          "A tuple of copies. Assign a list of JointConfig to change it.");
  value_type(config);

  nb::class_<JointState> state(m, "JointState");
  state.def_ro("position", &JointState::position, "rad")
      .def_ro("velocity", &JointState::velocity, "rad/s")
      .def_ro("load", &JointState::load, "-1 to 1 of the maximum PWM duty")
      .def_ro("current", &JointState::current, "A")
      .def_ro("temperature", &JointState::temperature, "C")
      .def_ro("servo_status", &JointState::servo_status, "feetech servo_error bits")
      .def_ro("torque_enabled", &JointState::torque_enabled)
      .def_ro("valid", &JointState::valid)
      .def_ro("sequence", &JointState::sequence)
      .def_ro("stamp_ns", &JointState::stamp_ns, "steady_clock ns");
  value_type(state);

  nb::enum_<specialist::Mode>(m, "Mode")
      .value("DISCONNECTED", specialist::Mode::kDisconnected)
      .value("CONNECTED", specialist::Mode::kConnected)
      .value("ACTIVE", specialist::Mode::kActive)
      .value("FAULTED", specialist::Mode::kFaulted);

  nb::enum_<specialist::Fault>(m, "Fault", nb::is_flag())
      .value("NONE", specialist::Fault::kNone)
      .value("BUS_LOST", specialist::Fault::kBusLost)
      .value("OVER_TEMPERATURE", specialist::Fault::kOverTemperature)
      .value("SERVO_RESET", specialist::Fault::kServoReset)
      .value("RELEASE_FAILED", specialist::Fault::kReleaseFailed);

  nb::class_<PyGripper>(m, "Gripper")
      .def(nb::init<>())
      .def_static(
          "validate",
          [](const GripperConfig& config) {
            std::string error;
            if (!Gripper::validate(config, error)) throw std::runtime_error(error);
          },
          "config"_a)
      .def("connect", raising<&Gripper::connect, const GripperConfig&>(), "config"_a, Blocking())
      .def("disconnect", &Gripper::disconnect, Blocking())
      .def("activate", raising<&Gripper::activate>(), Blocking())
      .def("deactivate", &Gripper::deactivate, Blocking())
      .def_prop_ro("mode", &Gripper::mode)
      .def_prop_ro("is_connected", &Gripper::is_connected)
      .def_prop_ro("is_active", &Gripper::is_active)
      .def_prop_ro("faulted", &Gripper::faulted)
      .def_prop_ro("fault", &Gripper::fault)
      .def_prop_ro("fault_message", &Gripper::fault_message)
      .def_prop_ro("last_error", &Gripper::last_error)
      .def_prop_ro("config", &Gripper::config, "A copy of the last validated configuration.")
      .def_prop_ro("joint_count", &Gripper::joint_count)
      .def("joint_index", &Gripper::joint_index, "name"_a)
      .def("set_target", indexed<&Gripper::set_target, double>(), "joint"_a, "radians"_a)
      .def("set_torque_enabled", indexed<&Gripper::set_torque_enabled, bool>(), "joint"_a,
           "enabled"_a)
      .def("set_torque_limit", indexed<&Gripper::set_torque_limit, double>(), "joint"_a,
           "fraction"_a)
      .def("state", indexed<&Gripper::state>(), "joint"_a)
      // Return the Python object: a returned reference would copy the noncopyable Gripper.
      .def(
          "__enter__", [](nb::object self) { return self; },
          nb::sig("def __enter__(self) -> Gripper"))
      .def(
          "__exit__",
          [](PyGripper& self, nb::handle, nb::handle, nb::handle) { self.disconnect(); },
          "exc_type"_a.none(), "exc_value"_a.none(), "traceback"_a.none(), Blocking());
}
