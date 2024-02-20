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

#include "Ipc.h"

#include "Codes.h"

using namespace dude::tf;

bool IPCBase::configuration(const YAML::Node &config) {
	if(is_running()) {
		// should not be configured when running
		return false;
	}

	m_is_configured = on_configuration(config);
	return m_is_configured;
}

bool IPCBase::init() {
	if(is_running()) {
		// should not be configured when running
		return false;
	}

	m_is_initialized = on_init();
	return m_is_initialized;
}

bool IPCBase::start() {
	if(is_running() or not is_initialized() or not is_configured()) {
		// must be in the correct state
		return false;
	}

	m_is_running = on_start();
	return m_is_running;
}

bool IPCBase::stop() {
	if(not is_running()) {
		// do nothing if not running
		return true;
	}

	if(on_stop()) {
		m_is_running = false;
		return true;
	} else {
		return false;
	}
}

bool IPCServerBase::on_remove_element(const dude::tf::id_t &id) {
	if(mp_callback_remove_element) {
		return mp_callback_remove_element(id);
	}
	return false;
}

Status_t IPCServerBase::on_write_element(const dude::tf::id_t &id,
                                         const std::optional<Frame> &frame,
                                         const std::optional<id_t> &parent) {
	if(mp_callback_write_element) {
		return mp_callback_write_element(id, frame, parent);
	}
	return {};
}

std::tuple<std::optional<Frame>, std::optional<dude::tf::id_t>, std::vector<dude::tf::id_t>,
           Status_t>
		IPCServerBase::on_read_element(const dude::tf::id_t &id, bool request_frame,
                                   bool request_parent, bool request_children) {
	if(mp_callback_read_element) {
		return mp_callback_read_element(id, request_frame, request_parent, request_children);
	}
	return {};
}

std::tuple<std::optional<Frame>, Status_t> IPCServerBase::on_read_path(const dude::tf::id_t &start,
                                                                       const dude::tf::id_t &end) {
	if(mp_callback_read_path) {
		return mp_callback_read_path(start, end);
	}
	return {};
}

bool IPCClientBase::add_element(const dude::tf::id_t &id, const Frame &frame,
                                const dude::tf::id_t &parent) {
	const auto &response_code = call_write_element(id, frame, parent);
	if(is_flag(response_code, StatusCodes::Success)) {
		return true;
	} else {
		// TODO: handle different status codes
		return false;
	}
}

bool IPCClientBase::element_exists(const dude::tf::id_t &id) {
	const auto &[_1, _2, _3, response_code] = call_read_element(id, false, false, false);
	return is_flag(response_code, StatusCodes::Success);
}
Frame IPCClientBase::get_element(const dude::tf::id_t &id) {
	const auto &[frame, _1, _2, response_code] = call_read_element(id, true, false, false);
	if(is_flag(response_code, StatusCodes::Success)) {
		if(frame.has_value()) {
			return frame.value();
		} else {
			// this should not happen...
			return Frame();
		}
	} else {
		// TODO: handle different status codes
		return Frame();
	}
}

bool IPCClientBase::update_element(const dude::tf::id_t &id, const Frame &new_frame) {
	const auto &response_code = call_write_element(id, new_frame);
	if(is_flag(response_code, StatusCodes::Success)) {
		return true;
	} else {
		// TODO: handle different status codes
		return false;
	}
}

bool IPCClientBase::move_element(const dude::tf::id_t &id, const dude::tf::id_t &new_parent) {
	const auto &response_code = call_write_element(id, new_parent);
	if(is_flag(response_code, StatusCodes::Success)) {
		return true;
	} else {
		// TODO: handle different status codes
		return false;
	}
}

bool IPCClientBase::remove_element(const dude::tf::id_t &id) {
	const auto &response_code = call_remove_element(id);
	if(is_flag(response_code, StatusCodes::Success)) {
		return true;
	} else {
		// TODO: handle different status codes
		return false;
	}
}

std::vector<dude::tf::id_t> IPCClientBase::get_roots() {
	const auto &[_1, _2, roots, response_code] = call_read_element(NO_ID, false, false, true);
	if(is_flag(response_code, StatusCodes::Success)) {
		return roots;
	} else {
		// TODO: handle different status codes
		return {};
	}
}
std::vector<dude::tf::id_t> IPCClientBase::get_children(const dude::tf::id_t &id) {
	const auto &[_1, _2, children, response_code] = call_read_element(id, false, false, true);
	if(is_flag(response_code, StatusCodes::Success)) {
		return children;
	} else {
		// TODO: handle different status codes
		return {};
	}
}
Frame IPCClientBase::get_path(const dude::tf::id_t &start, const dude::tf::id_t &end) {
	const auto &[path, response_code] = call_read_path(start, end);
	if(is_flag(response_code, StatusCodes::Success)) {
		if(path.has_value()) {
			return path.value();
		} else {
			// should not happen
			return Frame();
		}
	} else {
		// TODO: handle different status codes
		return Frame();
	}
}
bool IPCClientBase::path_exists(const dude::tf::id_t &start, const dude::tf::id_t &end) {
	const auto &[_1, response_code] = call_read_path(start, end);
	return is_flag(response_code, StatusCodes::Success);
}