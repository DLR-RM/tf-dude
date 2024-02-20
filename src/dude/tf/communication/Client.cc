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

#include "Client.h"
#include "Ipc.h"

#include <chrono>
#include <thread>
#include <utility>

using namespace std::chrono_literals;
using namespace dude::tf;

Client::Client(std::shared_ptr<IPCClientBase> ipc) : mp_ipc(std::move(ipc)) {}

Client::~Client() {
	if(is_running()) {
		// blocking call in the D'Tor... Well someone fucked up
		// not calling stop before this.
		stop(true);
	}
}

bool Client::configure(const YAML::Node &config) {
	if(mp_ipc) {
		m_is_configured = mp_ipc->configuration(config);
	}

	return m_is_configured;
}
bool Client::configure(const std::string &config_path) {
	if(mp_ipc) {
		m_is_configured = mp_ipc->configuration(config_path);
	}
	return m_is_configured;
}

bool Client::configure_default() {
	// empty yaml file; will use IPC-specific implementation
	return configure(YAML::Load(""));
}

bool Client::start() {
	if(not m_is_configured) {
		if(not configure_default()) {
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
void Client::stop(bool wait) {
	if(mp_ipc) {
		mp_ipc->stop();
		if(wait) {
			while(mp_ipc->is_running()) {
				std::this_thread::sleep_for(10ms);
			}
		}
	}
}

bool Client::is_running() const {
	if(mp_ipc) {
		return mp_ipc->is_running();
	}
	return false;
}

bool Client::add_element(const dude::tf::id_t &id, const Frame &frame,
                         const dude::tf::id_t &parent) {
	if(mp_ipc) {
		const auto &response_code = mp_ipc->call_write_element(id, frame, parent);
		if(is_flag(response_code, StatusCodes::Success)) {
			return true;
		} else {
			// TODO: inspect what is wrong
			return false;
		}
	}
	return false;
}

bool Client::element_exists(const dude::tf::id_t &id) {
	if(mp_ipc) {
		const auto &[frame, parent, children, response_code] =
				mp_ipc->call_read_element(id, false, false, false);

		// check if the success flag is set
		return is_flag(response_code, StatusCodes::Success);
	}
	return false;
}
Frame Client::get_element(const dude::tf::id_t &id) {
	if(mp_ipc) {
		return mp_ipc->get_element(id);
	}

	return Frame();
}
bool Client::update_element(const dude::tf::id_t &id, const Frame &new_frame) {
	if(mp_ipc) {
		return mp_ipc->update_element(id, new_frame);
	}
	return false;
}
bool Client::move_element(const dude::tf::id_t &id, const dude::tf::id_t &new_parent) {
	if(mp_ipc) {
		return mp_ipc->move_element(id, new_parent);
	}
	return false;
}
bool Client::remove_element(const dude::tf::id_t &id) {
	if(mp_ipc) {
		return mp_ipc->remove_element(id);
	}
	return false;
}
std::vector<dude::tf::id_t> Client::get_roots() {
	if(mp_ipc) {
		return mp_ipc->get_roots();
	}
	return std::vector<id_t>();
}
std::vector<dude::tf::id_t> Client::get_children(const dude::tf::id_t &parent) {
	if(mp_ipc) {
		return mp_ipc->get_children(parent);
	}
	return std::vector<id_t>();
}

Frame Client::get_path(const dude::tf::id_t &start, const dude::tf::id_t &end) {
	if(mp_ipc) {
		return mp_ipc->get_path(start, end);
	}
	return Frame();
}

bool Client::path_exists(const dude::tf::id_t &start, const dude::tf::id_t &end) {
	if(mp_ipc) {
		return mp_ipc->path_exists(start, end);
	}
	return false;
}
