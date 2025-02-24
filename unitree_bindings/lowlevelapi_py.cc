#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "src/lowlevelapi.h"
#include "src/lowlevelapi_types.h"


namespace py = pybind11;
using namespace pybind11::literals;


PYBIND11_MODULE(unitree_api, m) {
    m.doc() = "Unitree API bindings";

    py::class_<lowleveltypes::LowState>(m, "LowState")
        .def(py::init<>())
        .def_readwrite("foot_force", &lowleveltypes::LowState::foot_force);

    py::class_<lowleveltypes::IMUState>(m, "IMUState")
        .def(py::init<>())
        .def_readwrite("quaternion", &lowleveltypes::IMUState::quaternion)
        .def_readwrite("gyroscope", &lowleveltypes::IMUState::gyroscope)
        .def_readwrite("accelerometer", &lowleveltypes::IMUState::accelerometer)
        .def_readwrite("rpy", &lowleveltypes::IMUState::rpy);

    py::class_<lowleveltypes::MotorState>(m, "MotorState")
        .def(py::init<>())
        .def_readwrite("q", &lowleveltypes::MotorState::q)
        .def_readwrite("qd", &lowleveltypes::MotorState::qd)
        .def_readwrite("qdd", &lowleveltypes::MotorState::qdd)
        .def_readwrite("torque_estimate", &lowleveltypes::MotorState::torque_estimate)
        .def(py::pickle(
            [](const lowleveltypes::MotorState& obj) {
                return py::make_tuple(obj.q, obj.qd, obj.qdd, obj.torque_estimate);
            },
            [](py::tuple t) {
                if (t.size() != 4)
                    throw std::runtime_error("Invalid state tuple");
                lowleveltypes::MotorState obj;
                obj.q = t[0].cast<std::array<float, lowleveltypes::num_motors>>();
                obj.qd = t[1].cast<std::array<float, lowleveltypes::num_motors>>();
                obj.qdd = t[2].cast<std::array<float, lowleveltypes::num_motors>>();
                obj.torque_estimate = t[3].cast<std::array<float, lowleveltypes::num_motors>>();
                return obj;
            }
        ))
        .def("__copy__", [](const lowleveltypes::MotorState& obj) {
            return lowleveltypes::MotorState(obj);
        })
        .def("__deepcopy__", [](const lowleveltypes::MotorState& obj, py::dict) {
            return lowleveltypes::MotorState(obj);
        }, "memo"_a);

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
        .def("initialize", &MotorController::initialize)
        .def("stop_control_thread", &MotorController::stop_control_thread)
        .def("update_command", &MotorController::update_command)
        .def("get_low_state", &MotorController::get_low_state)
        .def("get_imu_state", &MotorController::get_imu_state)
        .def("get_motor_state", &MotorController::get_motor_state);
}
