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

#ifndef TF_DUDE_ROSCLIENT_H
#define TF_DUDE_ROSCLIENT_H

#include <dude/tf/communication/Ipc.h>
#include <dude/tf/core/Frame.h>
#include <dude/tf/core/Types.h>

#include <ros/ros.h>

#include <memory>
#include <optional>
#include <string>

namespace dude::tf::plugins {

class RosClient : public IPCClientBase {
protected:
	bool on_configuration(const YAML::Node &config) override;
	bool on_init() override;
	bool on_start() override;
	bool on_stop() override;

	Status_t call_write_element(const id_t &id, const std::optional<Frame> &frame,
	                            const std::optional<id_t> &parent) override;
	std::tuple<std::optional<Frame>, std::optional<id_t>, std::vector<id_t>, Status_t>
			call_read_element(const id_t &id, bool request_frame, bool request_parent,
	                      bool request_children) override;
	std::tuple<std::optional<Frame>, Status_t> call_read_path(const id_t &start,
	                                                          const id_t &end) override;
	Status_t call_remove_element(const id_t &id) override;

private:
	/* name of the ros node */
	std::string m_node_name;

	/* name of the server node */
	std::string m_server_name;

	/* ros node */
	std::shared_ptr<ros::NodeHandle> mp_handle;
};

}  // namespace dude::tf::plugins
#endif  // TF_DUDE_ROSCLIENT_H
