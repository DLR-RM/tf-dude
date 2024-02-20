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

#ifndef TF_DUDE_ROSSERVER_H
#define TF_DUDE_ROSSERVER_H

#include <dude/tf/communication/Ipc.h>

#include <ros/ros.h>

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace dude::tf::plugins {

class RosServer : public dude::tf::IPCServerBase {
protected:
	bool on_configuration(const YAML::Node &config) override;
	bool on_init() override;
	bool on_start() override;
	bool on_stop() override;

private:
	void background_thread();

	/* stop flag */
	std::atomic_bool m_stop_flag{false};

	/* background thread */
	std::thread m_background_thread;

	/* ros node */
	std::shared_ptr<ros::NodeHandle> mp_handle;

	/* ros node name */
	std::string m_node_name;
};

}  // namespace dude::tf::plugins
#endif  // TF_DUDE_ROSSERVER_H
