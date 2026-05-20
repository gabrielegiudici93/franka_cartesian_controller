#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include "cartesian_franka/robot.hpp"

namespace py = pybind11;

template<typename T>
    void declare_Interface(py::module &m, std::string typestr) {
	    using Class = franka_interface::RobotInterface<T>;
	    std::string pyclass_name = std::string("Robot") + typestr;
        std::cout<< "pyclass_name: " << pyclass_name << std::endl;
	    py::class_<Class>(m, pyclass_name.c_str(), py::buffer_protocol(), py::dynamic_attr())
	    .def(py::init<const std::string&, bool, bool, bool, double>(),
            py::arg("ip"),
            py::arg("realtime"),
            py::arg("hand_franka") = true,
            py::arg("auto_init") = true,
            py::arg("speed_factor") = 0.5)
        .def("init", &Class::init,py::arg("speed_factor") = 0.5)
        .def("translate", &Class::translate,py::arg("delta"),py::arg("duration") = 5)
        .def("rotate", &Class::rotate,py::arg("rpy"),py::arg("duration") = 5)
        .def("move", (void (Class::*)(const Eigen::Vector3d&, const Eigen::Vector3d&, double)) & Class::move,
            py::arg("delta"),
            py::arg("rpy"),
            py::arg("duration") = 5)
        .def("move", (void (Class::*)(const std::string& ,const Eigen::Matrix4d&, double)) & Class::extMove,
            py::arg("relative_or_absolute"),
            py::arg("m"),
            py::arg("duration") = 5)
        .def("move_joints", &Class::move_joints)
        .def("affine3d", &Class::affine3d)
        .def("position", &Class::position)
        .def("orientation", &Class::orientation)
        .def("automaticErrorRecovery", &Class::automaticErrorRecovery)
        .def("setFLController", &Class::setFLController)
        .def("setFLControllerTest", &Class::setFLControllerTest)
        .def("updateDesiredValues", &Class::updateDesiredValues)
        .def("getCtrlPerformance", &Class::getCtrlPerformance)
        .def("getCurState", &Class::getCurActionState)
        .def("setController",&Class::setController)
        .def("run",&Class::run)
        .def("runDetach",&Class::runDetach)
        .def("closeControlThread",&Class::closeControlThread)
        .def("getState",&Class::getSimpleState)
        .def("getgripState",&Class::getGripperSimpleState)
        .def("getFrame",&Class::getFrame)
        .def("moveGripper",&Class::moveGripper)
        .def("getMassMatrix", &Class::getMassMatrix, "Returns the current mass matrix.")
        .def("getCoriolisVector", &Class::getCoriolisVector, "Returns the current Coriolis vector.")
        .def("getGravityVector", &Class::getGravityVector, "Returns the current gravity vector.")
        // .def("getEndEffectorZeroJacobian", &Class::getEndEffectorZeroJacobian, "Returns the Jacobian for EndEffector.")
        .def("setUpdateFlags", &Class::setUpdateFlags)
        .def("grasp", &Class::grasp,
            py::arg("width"),
            py::arg("speed"),
            py::arg("force"),
            py::arg("epsilon_inner") = 0.005,  // Default value specified here
            py::arg("epsilon_outer") = 0.005,  // Default value specified here
            "Grasp method with optional epsilon_inner and epsilon_outer parameters")

        // added by luke
        .def("isError", &Class::isError)
        .def("getErrorStruct", &Class::getErrorStruct)
        .def("getErrorString", &Class::getErrorString)
        // end added by luke
        ;

    }


PYBIND11_MODULE(pyfranka_interface, m)
{
    m.doc() = "Basic Franka Emika interface, based on libfranka";

    using namespace franka_interface;
    // example of usage
    // import pyfranka_interface as franka
    //update_none = franka.UpdateFlags.UpdateNone
    //update_coriolis = franka.UpdateFlags.UpdateCoriolis
    //update_gravity = franka.UpdateFlags.UpdateGravity
    //update_mass = franka.UpdateFlags.UpdateMass

    //# You can use these values directly, for example, to set update flags
    
    //robot = franka.Robot_('172.16.0.2',False)
    //robot.set_update_flags(update_coriolis | update_mass)

     py::enum_<UpdateFlags>(m, "UpdateFlags")
      .value("UpdateNone", UpdateNone)
      .value("UpdateCoriolis", UpdateCoriolis)
      .value("UpdateGravity", UpdateGravity)
      .value("UpdateMass", UpdateMass)
      .export_values();

    py::class_<CtrlPerformance>(m, "CtrlPerformance")
		.def(py::init<>())
		.def_readwrite("q_des", &CtrlPerformance::q_des)
		.def_readwrite("q_err", &CtrlPerformance::q_err)
		.def_readwrite("qd_err", &CtrlPerformance::qd_err)
    .def_readwrite("last_tau", &CtrlPerformance::last_tau);

    py::class_<ReturnControlCommand>(m, "ReturnControlCommand")
		.def(py::init<>())
		.def_readwrite("joint_space_command", &ReturnControlCommand::joint_space_command)
		.def_readwrite("cartesian_pose_command", &ReturnControlCommand::cartesian_pose_command)
		.def_readwrite("cartesian_vel_command", &ReturnControlCommand::cartesian_vel_command)
		.def_readwrite("running_controller", &ReturnControlCommand::running_controller);

    py::class_<SimpleState>(m, "SimpleState")
		.def(py::init<>())
		.def_readwrite("q", &SimpleState::q)
		.def_readwrite("q_vel", &SimpleState::q_vel)
		.def_readwrite("tau", &SimpleState::tau)
		.def_property("T",[](SimpleState& self) ->  Eigen::Matrix4d& { return self.T; },[](SimpleState& self, Eigen::Matrix4d value) { self.T = value; })
		// .def_readwrite("J", &SimpleState::J)
		//.def_readwrite("T", &SimpleState::T)

        //def_read_write_mutable(cls, "T", &SimpleState::T)
        ;
    py::class_<GripSimpleState>(m, "GripSimpleState")
		.def(py::init<>())
		.def_readwrite("isgrasped", &GripSimpleState::isgrasped)
		.def_readwrite("width", &GripSimpleState::width)
		.def_readwrite("max_width", &GripSimpleState::max_width);  

    py::class_<franka_interface::Ref>(m, "empty")
    .def(py::init<>());    

    //The objectname in python will be robot_
    declare_Interface<void>(m, "_");

}
