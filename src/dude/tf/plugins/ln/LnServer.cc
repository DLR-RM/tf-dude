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

#include "LnServer.h"

#include "Handler.h"

using namespace dude::tf::plugins;

bool LnServer::on_configuration(const YAML::Node& config) {
	// get URI of ln manager instance
	if(config["ln_manager"]) {
		m_ln_manager = config["ln_manager"].as<std::string>();
	} else {
		std::cerr << "No ln_manager in configuration" << std::endl;
		return false;
	}

	// name of this client
	if(config["client_name"]) {
		m_client_name = config["client_name"].as<std::string>();
	} else {
		m_client_name = "tf_dude";
	}

	// add service handlers
	m_handler.push_back(std::shared_ptr<Handler>(new HandlerWriteElement));
	m_handler.push_back(std::shared_ptr<Handler>(new HandlerReadElement));
	m_handler.push_back(std::shared_ptr<Handler>(new HandlerReadPath));
	m_handler.push_back(std::shared_ptr<Handler>(new HandlerRemoveElement));
	return true;
}

bool LnServer::on_init() {
	return true;
}

bool LnServer::on_start() {
	// create client
	mp_client = std::make_shared<ln::client>(m_client_name, m_ln_manager);

	for(auto&& handler: m_handler) {
		if(not handler->init(mp_client.get(), this)) {
			return false;
		}
	}
	mp_client->handle_service_group_in_thread_pool("main_group", "main_pool");
	mp_client->set_max_threads("main_pool", 4);
	return true;
}

bool LnServer::on_stop() {
	if(mp_client) {
		for(auto&& handler: m_handler) {
			handler.reset();
		}

		mp_client.reset();
	}
	return true;
}
