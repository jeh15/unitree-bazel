#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "lowlevelapi.h"
#include "lowlevelapi_types.h"


namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_MODULE(unitree_api, m) {
    py::class_<lowleveltypes::MotorCommand>(m, "MotorCommand")
        .def(py::init<>())
        .def_readwrite("q_setpoint", &lowleveltypes::MotorCommand::q_setpoint)
        .def_readwrite("qd_setpoint", &lowleveltypes::MotorCommand::qd_setpoint)
        .def_readwrite("torque_feedforward", &lowleveltypes::MotorCommand::torque_feedforward)
        .def_readwrite("stiffness", &lowleveltypes::MotorCommand::stiffness)
        .def_readwrite("damping", &lowleveltypes::MotorCommand::damping)
        .def_readwrite("kp", &lowleveltypes::MotorCommand::kp)
        .def_readwrite("kd", &lowleveltypes::MotorCommand::kd);

    py::class_<MotorController>(m, "MotorController")
        .def(py::init<>())
        .def("init", &MotorController::init)
        .def("update_command", &MotorController::update_command)
        .def("get_low_state", &MotorController::get_low_state)
        .def("get_imu_state", &MotorController::get_imu_state)
        .def("get_motor_state", &MotorController::get_motor_state);
}
