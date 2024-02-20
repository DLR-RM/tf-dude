// *********************************************************
// tf_dude
// (c) 2022-2024
// Deutsches Zentrum fuer Luft- und Raumfahrt e.V.
// Institute fuer Robotik und Mechatronik
//
// German Aerospace Center
// Institute for Robotics and Mechatronics
//
// This file is part of the tf_dude repository and is provided as is.
// The repository's license does apply.
// 
// *********************************************************
//
// Authors:
// Sewtz, Marco
//
// *********************************************************

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <dude/tf/include.h>
#include <dude/tf/communication/Ipc.h>


using namespace dude::tf;
namespace py = pybind11;

Client create_client_with_plugin_name(const std::string& plugin_name) {
	return create_client(plugin_name);
}

Client create_client_default() {
	return create_client();
}

PYBIND11_MODULE(py_tf_dude, mod) {
	//
	// Frame class
	//
	py::class_<Frame>(mod, "Frame")
			// constructor with pose
			.def(py::init<Frame::PoseMatrix_t, Frame::CovMatrix_t>(),
	         py::arg("init_pose") = Frame::PoseMatrix_t::Identity(),
	         py::arg("init_cov")  = Frame::CovMatrix_t::Zero())
			// constructor with translation and quaternion
			.def(py::init<Frame::TranslationVector_t, std::array<Frame::MatrixType_t, 4>,
	                  Frame::CovMatrix_t>(),
	         py::arg("init_translation"),
	         py::arg("init_rotation") = std::array<Frame::MatrixType_t, 4>{1, 0, 0, 0},
	         py::arg("init_cov")      = Frame::CovMatrix_t::Zero())
			// pose functions
			.def("pose", py::overload_cast<>(&Frame::pose))
			.def("pose", py::overload_cast<>(&Frame::pose, py::const_))
			.def("set_pose", &Frame::set_pose, py::arg("new_pose"))
			// pose specific functions
			.def("set_translation", &Frame::set_translation, py::arg("new_translation"))
			.def("set_rotation", &Frame::set_rotation, py::arg("new_rotation"))
			// covariance functions
			.def("cov", py::overload_cast<>(&Frame::cov))
			.def("cov", py::overload_cast<>(&Frame::cov, py::const_))
			.def("set_covariance", &Frame::set_covariance, py::arg("new_cov"))
			// operators
			.def("compose", &Frame::compose, py::arg("other"))
			.def("inverse", &Frame::inverse);

	//
	// IPC abstract classes
	//
	py::class_<IPCBase, std::shared_ptr<IPCBase>>(mod, "IPCBase");
	py::class_<IPCClientBase, std::shared_ptr<IPCClientBase>>(mod, "IPCClientBase");
	py::class_<IPCServerBase, std::shared_ptr<IPCServerBase>>(mod, "IPCServerBase");

	//
	// Client Class
	//
	py::class_<Client>(mod, "Client")
			// constructor
			.def(py::init<std::shared_ptr<IPCClientBase>>(), py::arg("ipc"))
			// configuration functions
			.def("configure", py::overload_cast<const YAML::Node&>(&Client::configure))
			.def("configure", py::overload_cast<const std::string&>(&Client::configure))
			.def("configure_default", &Client::configure_default)
			// client control functions
			.def("start", &Client::start)
			.def("stop", &Client::stop)
			.def("is_running", &Client::is_running)
			// tf_dude communication functions
			.def("add_element", &Client::add_element)
			.def("element_exists", &Client::element_exists)
			.def("get_element", &Client::get_element)
			.def("update_element", &Client::update_element)
			.def("move_element", &Client::move_element)
			.def("remove_element", &Client::remove_element)
			.def("get_roots", &Client::get_roots)
			.def("get_children", &Client::get_children)
			.def("get_path", &Client::get_path)
			.def("path_exists", &Client::path_exists);

	//
	// integration utilities
	//
	mod.def("get_supported_plugins", &get_supported_plugins);
	mod.def("get_default_plugin", &get_default_plugin);
	mod.def("create_client", &create_client_with_plugin_name);
	mod.def("create_client", &create_client_default);
}