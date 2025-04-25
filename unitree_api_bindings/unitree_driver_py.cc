#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "pybind11_abseil/absl_casters.h"
#include "pybind11_abseil/status_caster.h"
#include "pybind11_abseil/import_status_module.h"

#include "unitree-api/unitree_driver.h"
#include "unitree-api/containers.h"


namespace py = pybind11;
using namespace pybind11::literals;


PYBIND11_MODULE(unitree_api, m) {
    py::google::ImportStatusModule();

    m.doc() = "Unitree API bindings";

    py::class_<unitree::containers::LowState>(m, "LowState")
        .def(py::init<>())
        .def_readwrite("foot_force", &unitree::containers::LowState::foot_force);

    py::class_<unitree::containers::IMUState>(m, "IMUState")
        .def(py::init<>())
        .def_readwrite("quaternion", &unitree::containers::IMUState::quaternion)
        .def_readwrite("gyroscope", &unitree::containers::IMUState::gyroscope)
        .def_readwrite("accelerometer", &unitree::containers::IMUState::accelerometer)
        .def_readwrite("rpy", &unitree::containers::IMUState::rpy);

    py::class_<unitree::containers::MotorState>(m, "MotorState")
        .def(py::init<>())
        .def_readwrite("q", &unitree::containers::MotorState::q)
        .def_readwrite("qd", &unitree::containers::MotorState::qd)
        .def_readwrite("qdd", &unitree::containers::MotorState::qdd)
        .def_readwrite("torque_estimate", &unitree::containers::MotorState::torque_estimate)
        .def(py::pickle(
            [](const unitree::containers::MotorState& obj) {
                return py::make_tuple(obj.q, obj.qd, obj.qdd, obj.torque_estimate);
            },
            [](py::tuple t) {
                if (t.size() != 4)
                    throw std::runtime_error("Invalid state tuple");
                unitree::containers::MotorState obj;
                obj.q = t[0].cast<std::array<float, unitree::containers::num_motors>>();
                obj.qd = t[1].cast<std::array<float, unitree::containers::num_motors>>();
                obj.qdd = t[2].cast<std::array<float, unitree::containers::num_motors>>();
                obj.torque_estimate = t[3].cast<std::array<float, unitree::containers::num_motors>>();
                return obj;
            }
        ))
        .def("__copy__", [](const unitree::containers::MotorState& obj) {
            return unitree::containers::MotorState(obj);
        })
        .def("__deepcopy__", [](const unitree::containers::MotorState& obj, py::dict) {
            return unitree::containers::MotorState(obj);
        }, "memo"_a);

    py::class_<unitree::containers::MotorCommand>(m, "MotorCommand")
        .def(py::init<>())
        .def_readwrite("q_setpoint", &unitree::containers::MotorCommand::q_setpoint)
        .def_readwrite("qd_setpoint", &unitree::containers::MotorCommand::qd_setpoint)
        .def_readwrite("torque_feedforward", &unitree::containers::MotorCommand::torque_feedforward)
        .def_readwrite("stiffness", &unitree::containers::MotorCommand::stiffness)
        .def_readwrite("damping", &unitree::containers::MotorCommand::damping);

    py::class_<UnitreeDriver>(m, "UnitreeDriver")
        .def(py::init<const std::string, int>())
        .def("initialize", &UnitreeDriver::initialize)
        .def("initialize_thread", &UnitreeDriver::initialize_thread)
        .def("stop_thread", &UnitreeDriver::stop_thread)
        .def("update_command", &UnitreeDriver::update_command)
        .def("get_low_state", &UnitreeDriver::get_low_state)
        .def("get_imu_state", &UnitreeDriver::get_imu_state)
        .def("get_motor_state", &UnitreeDriver::get_motor_state);
}
