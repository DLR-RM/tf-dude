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

#include "LnClient.h"
#include "Utils.h"

#include <ln/ln_messages.h>

#include <iostream>

using namespace dude::tf;
using namespace dude::tf::plugins;

bool LnClient::on_configuration(const YAML::Node &config) {
	// get URI of ln manager instance
	if(config["ln_manager"]) {
		m_ln_manager = config["ln_manager"].as<std::string>();
	} else {
		return false;
	}

	// name of this client
	if(config["client_name"]) {
		m_client_name = config["client_name"].as<std::string>();
	} else {
		return false;
	}

	return true;
}

bool LnClient::on_init() {
	mp_client = std::make_shared<ln::client>(m_client_name, m_ln_manager);

	return true;
}

bool LnClient::on_start() {
	// we do not have to connect again

	return true;
}

bool LnClient::on_stop() {
	return true;
}

std::shared_ptr<ln::service> LnClient::get_service(const std::string &service_name) {
	if(mp_client) {
		// retrieve signature
		std::string service_interface;
		std::string definition;
		std::string signature;
		mp_client->get_service_signature_for_service(service_name, service_interface, definition,
		                                             signature);

		// get service instance
		return std::shared_ptr<ln::service>(
				mp_client->get_service(service_name, service_interface, signature));
	}
	return {};
}

Status_t LnClient::call_remove_element(const dude::tf::id_t &id) {
	// fill service
	dude::tf::requests::remove_element_t data{};
	data.req.id = utils::to_ln_id(id);

	// call service
	auto service = get_service("dude.tf.remove_element");
	service->call(&data, TIMEOUT);

	return data.resp.response_code;
}

dude::tf::Status_t LnClient::call_write_element(const dude::tf::id_t &id,
                                                const std::optional<Frame> &frame,
                                                const std::optional<id_t> &parent) {
	// fill service
	dude::tf::requests::write_element_t data{};

	Status_t status{};
	data.req.element_id = utils::to_ln_id(id);
	if(frame.has_value()) {
		data.req.frame = utils::to_ln_frame(frame.value());
		status         = set_flag(status, StatusCodes::Frame);
	}
	if(parent.has_value()) {
		data.req.parent = utils::to_ln_id(parent.value());
		status          = set_flag(status, StatusCodes::Parent);
	}
	data.req.status = status;

	auto service = get_service("dude.tf.write_element");
	service->call(&data, TIMEOUT);

	return data.resp.response_code;
}
std::tuple<std::optional<Frame>, std::optional<dude::tf::id_t>, std::vector<dude::tf::id_t>,
           Status_t>
		LnClient::call_read_element(const dude::tf::id_t &id, bool request_frame, bool request_parent,
                                bool request_children) {
	// fill service
	dude::tf::requests::read_element_t data{};
	data.req.id = utils::to_ln_id(id);
	Status_t status{};
	if(request_frame) {
		status = set_flag(status, StatusCodes::Frame);
	}
	if(request_parent) {
		status = set_flag(status, StatusCodes::Parent);
	}
	if(request_children) {
		status = set_flag(status, StatusCodes::Children);
	}
	data.req.status = status;

	auto service = get_service("dude.tf.read_element");
	service->call(&data, TIMEOUT);

	std::optional<Frame> frame;
	std::optional<id_t> parent;
	std::vector<id_t> children;
	if(is_flag(data.resp.response_code, StatusCodes::Success | StatusCodes::Frame)) {
		frame = utils::to_dude_frame(data.resp.frame);
	}
	if(is_flag(data.resp.response_code, StatusCodes::Success | StatusCodes::Parent)) {
		parent = utils::to_dude_id(data.resp.parent);
	}
	if(is_flag(data.resp.response_code, StatusCodes::Success | StatusCodes::Children)) {
		for(std::size_t i = 0; i < data.resp.children_len; ++i) {
			children.emplace_back(utils::to_dude_id(data.resp.children[i]));
		}
	}

	return {frame, parent, children, Status_t{data.resp.response_code}};
}
std::tuple<std::optional<Frame>, Status_t> LnClient::call_read_path(const dude::tf::id_t &start,
                                                                    const dude::tf::id_t &end) {
	// fill service
	dude_tf_requests_read_path_t data{};
	data.req.start = utils::to_ln_id(start);
	data.req.end   = utils::to_ln_id(end);

	auto service = get_service("dude.tf.read_path");
	service->call(&data, TIMEOUT);

	std::optional<Frame> frame;
	if(is_flag(data.resp.response_code, StatusCodes::Success)) {
		frame = utils::to_dude_frame(data.resp.path_frame);
	}
	return {frame, Status_t{data.resp.response_code}};
}
