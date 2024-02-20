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

#include "RosClient.h"
#include "Utils.h"

#include <tf_dude/read_element.h>
#include <tf_dude/read_path.h>
#include <tf_dude/remove_element.h>
#include <tf_dude/write_element.h>

#include <random>

using namespace dude::tf;
using namespace plugins;

bool RosClient::on_configuration(const YAML::Node &config) {
	// name of this client
	if(config["node_name"]) {
		m_node_name = config["node_name"].as<std::string>();
	} else {
		// we have to generate a client name
		std::random_device dev;
		std::mt19937_64 random(dev());
		std::uniform_int_distribution<uint64_t> distribution;
		m_node_name = "tf_dude_client_" + std::to_string(distribution(random));
	}
	// name of the server
	if(config["server_name"]) {
		m_node_name = config["server_name"].as<std::string>();
	} else {
		m_server_name = "tf_dude";
	}
	return true;
}
bool RosClient::on_init() {
	// TODO: add argc, argv to init somehow
	int fake_argc = 0;
	ros::init(fake_argc, nullptr, m_node_name, ros::init_options::NoSigintHandler);

	return ros::isInitialized();
}
bool RosClient::on_start() {
	// register handlers
	mp_handle = std::make_shared<ros::NodeHandle>();
	return true;
}
bool RosClient::on_stop() {
	mp_handle.reset();
	return true;
}
Status_t RosClient::call_write_element(const dude::tf::id_t &id, const std::optional<Frame> &frame,
                                       const std::optional<dude::tf::id_t> &parent) {
	// create service
	auto service_client =
			mp_handle->serviceClient<tf_dude::write_element>("/" + m_server_name + "/write_element");

	// fill service
	tf_dude::write_element service{};
	Status_t status_code{};
	service.request.element_id = id;
	if(frame.has_value()) {
		status_code           = set_flag(status_code, StatusCodes::Frame);
		service.request.frame = utils::to_ros_frame(frame.value());
	}
	if(parent.has_value()) {
		status_code            = set_flag(status_code, StatusCodes::Parent);
		service.request.parent = parent.value();
	}
	service.request.status = status_code;

	// call
	service_client.call(service);
	return service.response.response_code;
}
std::tuple<std::optional<Frame>, std::optional<dude::tf::id_t>, std::vector<dude::tf::id_t>,
           Status_t>
		RosClient::call_read_element(const dude::tf::id_t &id, bool request_frame, bool request_parent,
                                 bool request_children) {
	// create service
	auto service_client =
			mp_handle->serviceClient<tf_dude::read_element>("/" + m_server_name + "/read_element");

	// fill service
	tf_dude::read_element service{};
	Status_t status_code{};
	service.request.element_id = id;
	if(request_frame) {
		status_code = set_flag(status_code, StatusCodes::Frame);
	}
	if(request_parent) {
		status_code = set_flag(status_code, StatusCodes::Parent);
	}
	if(request_children) {
		status_code = set_flag(status_code, StatusCodes::Children);
	}
	service.request.status = status_code;

	// call
	service_client.call(service);

	// response
	std::optional<Frame> frame;
	std::optional<dude::tf::id_t> parent;
	std::vector<dude::tf::id_t> children;
	if(is_flag(service.response.response_code, StatusCodes::Frame)) {
		frame = utils::to_dude_frame(service.response.frame);
	}
	if(is_flag(service.response.response_code, StatusCodes::Parent)) {
		parent = service.response.parent;
	}
	if(is_flag(service.response.response_code, StatusCodes::Children)) {
		children = service.response.children;
	}

	return {frame, parent, children, service.response.response_code};
}
std::tuple<std::optional<Frame>, Status_t> RosClient::call_read_path(const dude::tf::id_t &start,
                                                                     const dude::tf::id_t &end) {
	// create service
	auto service_client =
			mp_handle->serviceClient<tf_dude::read_path>("/" + m_server_name + "/read_path");

	// fill service
	tf_dude::read_path service{};
	service.request.start = start;
	service.request.stop  = end;  // TODO: align naming here

	// call
	service_client.call(service);

	// response
	std::optional<Frame> path;
	if(is_flag(service.response.response_code, StatusCodes::Success)) {
		path = utils::to_dude_frame(service.response.path);
	}
	return {path, service.response.response_code};
}
Status_t RosClient::call_remove_element(const dude::tf::id_t &id) {
	// create service
	auto service_client =
			mp_handle->serviceClient<tf_dude::remove_element>("/" + m_server_name + "/remove_element");

	// fill service
	tf_dude::remove_element service{};
	service.request.element_id = id;

	// call
	service_client.call(service);
	return service.response.response_code;
}