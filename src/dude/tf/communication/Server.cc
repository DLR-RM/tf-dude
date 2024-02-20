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

#include "Server.h"

#include "Ipc.h"

#include <chrono>
#include <thread>
#include <utility>

using namespace std::chrono_literals;
using namespace dude::tf;

Server::Server(std::shared_ptr<IPCServerBase> ipc) : mp_ipc(std::move(ipc)) {
	// register callbacks
	mp_ipc->add_server_handle(this);
}

bool Server::configure(const YAML::Node &config) {
	if(mp_ipc) {
		m_is_configured = mp_ipc->configuration(config);
	}
	return m_is_configured;
}

bool Server::configure(const std::string &config_path) {
	if(mp_ipc) {
		m_is_configured = mp_ipc->configuration(config_path);
	}
	return m_is_configured;
}

bool Server::configure_default() {
	// empty yaml file; will use IPC-specific implementation
	return configure(YAML::Load(""));
}

bool Server::start() {
	// check if we have to configure with default values
	if(not m_is_configured){
		if(not configure_default()){
			return false;
		}
	}

	if(mp_ipc) {
		// run the whole lifecycle
		if(not mp_ipc->init()) {
			return false;
		}
		if(not mp_ipc->start()) {
			return false;
		}
		return true;
	}
	return false;
}

void Server::stop(bool wait) {
	if(mp_ipc) {
		mp_ipc->stop();
		if(wait) {
			while(mp_ipc->is_running()) {
				std::this_thread::sleep_for(10ms);
			}
		}
	}
}

bool Server::is_running() const {
	if(mp_ipc) {
		return mp_ipc->is_running();
	}
	return false;
}

Status_t Server::on_remove_element(const dude::tf::id_t &id) {
	const auto &result = m_tree.remove_element(id);

	Status_t response_code{};
	if(result) {
		response_code = set_flag(response_code, StatusCodes::Success);
	}
	return response_code;
}

Status_t Server::on_write_element(const dude::tf::id_t &id, const std::optional<Frame> &frame,
                                  const std::optional<id_t> &parent) {
	Status_t response_code{};

	// check if the element exists
	if(not m_tree.exists(id)) {
		// this is a new element. We have to add it
		if(not frame.has_value() or not parent.has_value()) {
			// a new element requires a frame and a parent
			response_code = unset_flag(response_code, StatusCodes::Success);
			std::cerr << "New element needs parent and frame to be set" << std::endl;
			return response_code;
		}

		// add element
		m_tree.add_element(id, frame.value(), parent.value());
	} else {
		// update fields
		if(parent.has_value()) {
			m_tree.move_element(id, parent.value());
			response_code = set_flag(response_code, StatusCodes::Parent);
		}
		if(frame.has_value()) {
			m_tree.get(id) = frame.value();
			response_code  = set_flag(response_code, StatusCodes::Frame);
		}
	}

	// set the success flag
	response_code = set_flag(response_code, StatusCodes::Success);
	return response_code;
}

std::tuple<std::optional<Frame>, std::optional<dude::tf::id_t>, std::vector<dude::tf::id_t>,
           Status_t>
		Server::on_read_element(const dude::tf::id_t &id, bool request_frame, bool request_parent,
                            bool request_children) {
	std::optional<Frame> frame;
	std::optional<id_t> parent;
	std::vector<id_t> children;
	Status_t response_code{};

	// return roots if id is NO_ID and children are requested
	if(id == NO_ID and request_children) {
		children      = m_tree.get_all_roots();
		response_code = set_flag(response_code, StatusCodes::Success | StatusCodes::Children);
		return {frame, parent, children, response_code};
	} else {
		// return if requested element doesn't exist
		if(not m_tree.exists(id)) {
			response_code = unset_flag(response_code, StatusCodes::Success);
			return {frame, parent, children, response_code};
		}

		if(request_frame) {
			frame         = m_tree.get(id);
			response_code = set_flag(response_code, StatusCodes::Frame);
		}

		if(request_parent) {
			parent        = m_tree.get_parent_id(id);
			response_code = set_flag(response_code, StatusCodes::Parent);
		}

		if(request_children) {
			children      = m_tree.get_all_children(id);
			response_code = set_flag(response_code, StatusCodes::Children);
		}

		response_code = set_flag(response_code, StatusCodes::Success);
		return {frame, parent, children, response_code};
	}
}

std::tuple<std::optional<Frame>, Status_t> Server::on_read_path(const dude::tf::id_t &start,
                                                                const dude::tf::id_t &end) {
	std::optional<Frame> frame;
	Status_t response_code{};

	// check start & end
	const bool &start_exists = m_tree.exists(start);
	const bool &end_exists   = m_tree.exists(end);
	if(start_exists) {
		response_code = set_flag(response_code, StatusCodes::Start);
	}
	if(end_exists) {
		response_code = set_flag(response_code, StatusCodes::End);
	}
	if(not(start_exists and end_exists)) {
		response_code = unset_flag(response_code, StatusCodes::Success);
		return {Frame(), response_code};
	}

	// get path
	const auto &path = m_tree.get_composed_transform(start, end);
	response_code    = set_flag(response_code, StatusCodes::Success);
	return {path, response_code};
}
