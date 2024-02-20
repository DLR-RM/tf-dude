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

#include "Handler.h"
#include "Utils.h"

#include <dude/tf/communication/Codes.h>

#include <optional>

using namespace dude::tf;
using namespace dude::tf::plugins;

bool Handler::register_handler(IPCServerBase *server,
                               const std::shared_ptr<ros::NodeHandle> &handle) {
	// set a pointer to server instance
	mp_server = std::shared_ptr<IPCServerBase>(server);

	m_services.push_back(
			handle->advertiseService("tf_dude/read_element", &Handler::handler_read_element, this));
	m_services.push_back(
			handle->advertiseService("tf_dude/write_element", &Handler::handler_write_element, this));
	m_services.push_back(
			handle->advertiseService("tf_dude/remove_element", &Handler::handler_remove_element, this));
	m_services.push_back(
			handle->advertiseService("tf_dude/read_path", &Handler::handler_read_path, this));

	return true;
}

void Handler::unregister_handler() {
	m_services.clear();
	mp_server.reset();
}

bool Handler::handler_write_element(tf_dude::write_element::Request &req,
                                    tf_dude::write_element::Response &resp) {
	std::cout << "Received write element" << std::endl;
	if(mp_server) {
		const dude::tf::id_t &element_id = req.element_id;
		std::optional<dude::tf::id_t> parent_id;
		std::optional<Frame> frame;
		if(is_flag(req.status, StatusCodes::Parent)) {
			parent_id = req.parent;
		}
		if(is_flag(req.status, StatusCodes::Frame)) {
			frame = utils::to_dude_frame(req.frame);
		}

		const auto &response_code = mp_server->on_write_element(element_id, frame, parent_id);
		std::cout << std::to_string(response_code) << std::endl;
		resp.response_code = response_code;
		return true;
	}

	return false;
}

bool Handler::handler_read_element(tf_dude::read_element::Request &req,
                                   tf_dude::read_element::Response &resp) {
	std::cout << "Received read element" << std::endl;
	if(mp_server) {
		const dude::tf::id_t &element_id = req.element_id;

		const auto &[frame, parent, children, response_code] = mp_server->on_read_element(
				element_id, is_flag(req.status, StatusCodes::Frame),
				is_flag(req.status, StatusCodes::Parent), is_flag(req.status, StatusCodes::Children));

		resp.response_code = response_code;
		if(frame.has_value()) {
			resp.frame = utils::to_ros_frame(frame.value());
		}
		if(parent.has_value()) {
			resp.parent = parent.value();
		}
		if(not children.empty()) {
			resp.children = children;
		}
		return true;
	}
	return false;
}

bool Handler::handler_remove_element(tf_dude::remove_element::Request &req,
                                     tf_dude::remove_element::Response &resp) {
	std::cout << "Received remove element" << std::endl;
	if(mp_server) {
		const dude::tf::id_t &element_id = req.element_id;
		Status_t response_code;

		if(mp_server->on_remove_element(element_id)) {
			response_code = set_flag(response_code, StatusCodes::Success);
		}
		resp.response_code = response_code;
		return true;
	}
	return false;
}

bool Handler::handler_read_path(tf_dude::read_path::Request &req,
                                tf_dude::read_path::Response &resp) {
	std::cout << "Received read path" << std::endl;
	if(mp_server) {
		const dude::tf::id_t &start = req.start;
		const dude::tf::id_t &stop  = req.stop;

		const auto &[path, response_code] = mp_server->on_read_path(start, stop);
		if(path.has_value()) {
			resp.path = utils::to_ros_frame(path.value());
		}
		resp.response_code = response_code;
		return true;
	}
	return false;
}
