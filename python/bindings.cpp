// Grippers(...) raises ValueError for a config it refuses. `with grippers:` connects, and raises
// RuntimeError with the error when that fails; its end disconnects.

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <stdexcept>
#include <string>
#include <vector>

#include "specialist/gripper.hpp"

namespace nb = nanobind;
using namespace nb::literals;
using specialist::Gripper, specialist::GripperConfig, specialist::GripperState,
    specialist::Grippers;

NB_MODULE(_core, m) {
  nb::class_<GripperConfig>(m, "GripperConfig")
      .def(nb::init<>())
      .def(nb::init<const GripperConfig&>(), "other"_a)
      .def_rw("name", &GripperConfig::name)
      .def_rw("port", &GripperConfig::port)
      .def_rw("id", &GripperConfig::id)
      .def_rw("closed", &GripperConfig::closed, "rad")
      .def_rw("open", &GripperConfig::open, "rad, the open stop")
      .def_rw("torque_limit", &GripperConfig::torque_limit, "permille of stall")
      .def_rw("direction", &GripperConfig::direction, "+1 or -1; NaN until calibrated")
      .def_rw("offset", &GripperConfig::offset, "rad; NaN until calibrated");

  nb::class_<GripperState>(m, "GripperState")
      .def_ro("openness", &GripperState::openness, "rad")
      .def_ro("velocity", &GripperState::velocity, "rad/s")
      .def_ro("load", &GripperState::load, "-1 to 1 of the maximum PWM duty, positive when opening")
      .def_ro("current", &GripperState::current, "A")
      .def_ro("temperature", &GripperState::temperature, "C")
      .def_ro("torque_enabled", &GripperState::torque_enabled)
      .def_ro("valid", &GripperState::valid)
      .def_ro("sequence", &GripperState::sequence);

  nb::class_<Gripper>(m, "Gripper", "From Grippers[name].")
      .def("set_target", &Gripper::set_target, "openness"_a, "rad; turns the torque on")
      .def("release", &Gripper::release)
      .def("state", &Gripper::state);

  nb::class_<Grippers>(m, "Grippers")
      .def(nb::init<std::vector<GripperConfig>>(), "grippers"_a)
      .def_prop_ro("faulted", &Grippers::faulted)
      .def_prop_ro("error", &Grippers::error)
      // The Gripper keeps the Grippers alive.
      .def(
          "__getitem__",
          [](Grippers& self, const std::string& name) {
            if (auto gripper = self.gripper(name)) return *gripper;
            throw nb::key_error(("no gripper '" + name + "'").c_str());
          },
          "name"_a, nb::keep_alive<0, 1>())
      .def(
          "__enter__",
          [](nb::object self) {
            Grippers& grippers = nb::cast<Grippers&>(self);
            bool connected;
            {
              nb::gil_scoped_release released;
              connected = grippers.connect();
            }
            if (!connected) throw std::runtime_error(grippers.error());
            return self;
          },
          nb::sig("def __enter__(self) -> Grippers"))
      .def(
          "__exit__",
          [](Grippers& self, nb::handle, nb::handle, nb::handle) { (void)self.disconnect(); },
          "exc_type"_a.none(), "exc_value"_a.none(), "traceback"_a.none(),
          nb::call_guard<nb::gil_scoped_release>());
}
