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

#include "include.h"

#include <dude/tf/plugins/build_options.h>

#ifdef BUILD_MODULE_TF_PLUGINS_LN
#include <dude/tf/plugins/ln/LnClient.h>
#include <dude/tf/plugins/ln/LnServer.h>
#endif
#ifdef BUILD_MODULE_TF_PLUGINS_ROS
#include <dude/tf/plugins/ros/RosClient.h>
#include <dude/tf/plugins/ros/RosServer.h>
#endif

#include <iostream>

std::vector<std::string> dude::tf::get_supported_plugins() {
	std::vector<std::string> plugins;

#ifdef BUILD_MODULE_TF_PLUGINS_LN
	plugins.emplace_back("ln");
#endif

#ifdef BUILD_MODULE_TF_PLUGINS_ROS
	plugins.emplace_back("ros");
#endif

#ifdef BUILD_MODULE_TF_PLUGINS_ROS2
	plugins.emplace_back("ros2");
#endif

	return plugins;
}

std::string dude::tf::get_default_plugin() {
	return {DEFAULT_PLUGIN};
}

dude::tf::Client dude::tf::create_client(const std::optional<std::string> &plugin_name) {
	// use default if nothing is set
	if(not plugin_name.has_value()) {
		return create_client(get_default_plugin());
	}

	// check if plugin is supported
	const auto &supported = get_supported_plugins();
	if(std::find(supported.begin(), supported.end(), plugin_name) == supported.end()) {
		std::cerr << "Unsupported plugin '" << plugin_name.value() << "'" << std::endl;
		return Client(nullptr);
	}

	// return the correct client
#ifdef BUILD_MODULE_TF_PLUGINS_LN
	if(plugin_name == "ln") {
		return Client(std::shared_ptr<IPCClientBase>(new plugins::LnClient));
	}
#endif
#ifdef BUILD_MODULE_TF_PLUGINS_ROS
	if(plugin_name == "ros") {
		return Client(std::shared_ptr<IPCClientBase>(new plugins::RosClient));
	}
#endif

	std::cerr << "Unsupported plugin '" << plugin_name.value() << "'" << std::endl;
	return Client(nullptr);
}
dude::tf::Server dude::tf::create_server(const std::optional<std::string> &plugin_name) {
	// use default if nothing is set
	if(not plugin_name.has_value()) {
		return create_server(get_default_plugin());
	}

	// check if plugin is supported
	const auto &supported = get_supported_plugins();
	if(std::find(supported.begin(), supported.end(), plugin_name) == supported.end()) {
		std::cerr << "Unsupported plugin '" << plugin_name.value() << "'" << std::endl;
		return Server(nullptr);
	}

	// return the correct client
#ifdef BUILD_MODULE_TF_PLUGINS_LN
	if(plugin_name == "ln") {
		return Server(std::shared_ptr<IPCServerBase>(new plugins::LnServer));
	}
#endif
#ifdef BUILD_MODULE_TF_PLUGINS_ROS
	if(plugin_name == "ros") {
		return Server(std::shared_ptr<IPCServerBase>(new plugins::RosServer));
	}
#endif

	std::cerr << "Unsupported plugin '" << plugin_name.value() << "'" << std::endl;
	return Server(nullptr);
}
