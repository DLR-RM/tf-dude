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

#include "RosServer.h"
#include "Handler.h"

#include <chrono>

using namespace dude::tf;
using namespace dude::tf::plugins;
using namespace std::chrono_literals;

Handler ros_handler;

bool RosServer::on_configuration(const YAML::Node &config) {
	// name of this client
	if(config["node_name"]) {
		m_node_name = config["node_name"].as<std::string>();
	} else {
		m_node_name = "tf_dude";
	}
	return true;
}
bool RosServer::on_init() {
	// TODO: add argc, argv to init somehow
	int fake_argc = 0;
	ros::init(fake_argc, nullptr, m_node_name, ros::init_options::NoSigintHandler);

	// register handlers
	mp_handle = std::make_shared<ros::NodeHandle>();
	ros_handler.register_handler(this, mp_handle);

	return ros::isInitialized();
}
bool RosServer::on_start() {
	m_stop_flag = false;

	// start thread
	m_background_thread = std::thread(&RosServer::background_thread, this);

	return true;
}
bool RosServer::on_stop() {
	m_stop_flag = true;

	if(m_background_thread.joinable()) {
		m_background_thread.join();
	}
	return true;
}

void RosServer::background_thread() {
	ros::Rate rate(100);
	while(not m_stop_flag) {
		ros::spinOnce();
		rate.sleep();
	}
}