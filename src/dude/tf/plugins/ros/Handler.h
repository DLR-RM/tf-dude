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

#ifndef TF_DUDE_HANDLER_H
#define TF_DUDE_HANDLER_H

#include <dude/tf/communication/Ipc.h>

#include <tf_dude/read_element.h>
#include <tf_dude/read_path.h>
#include <tf_dude/remove_element.h>
#include <tf_dude/write_element.h>

#include <ros/ros.h>

#include <memory>
#include <vector>

namespace dude::tf::plugins {

class Handler {
public:
	bool register_handler(IPCServerBase* server, const std::shared_ptr<ros::NodeHandle>& handle);

	void unregister_handler();

private:
	std::shared_ptr<IPCServerBase> mp_server;

	/* vector with ros services */
	std::vector<ros::ServiceServer> m_services;

	bool handler_read_path(tf_dude::read_path::Request& req, tf_dude::read_path::Response& resp);

	bool handler_read_element(tf_dude::read_element::Request& req,
	                          tf_dude::read_element::Response& resp);

	bool handler_write_element(tf_dude::write_element::Request& req,
	                           tf_dude::write_element::Response& resp);

	bool handler_remove_element(tf_dude::remove_element::Request& req,
	                            tf_dude::remove_element::Response& resp);
};

}  // namespace dude::tf::plugins
#endif  // TF_DUDE_HANDLER_H
