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

#include "Handler.h"
#include "Utils.h"

#include <dude/tf/communication/Ipc.h>

#include <iostream>

using namespace dude::tf::plugins;

bool Handler::init(ln::client* client, dude::tf::IPCServerBase* p_server) {
	mp_server = p_server;
	return this->on_init(client);
}

bool HandlerRemoveElement::on_init(ln::client* client) {
	this->register_remove_element(client, "dude.tf.remove_element", "main_group");
	return true;
}
HandlerRemoveElement::~HandlerRemoveElement() {
	this->unregister_remove_element();
}
int HandlerRemoveElement::on_remove_element(ln::service_request& req,
                                            dude::tf::requests::remove_element_t& data) {
	Status_t response_code{};
	const auto id        = utils::to_dude_id(data.req.id);
	const auto& response = server().on_remove_element(id);
	if(response) {
		response_code = set_flag(response_code, StatusCodes::Success);
	} else {
		response_code = unset_flag(response_code, StatusCodes::Success);
	}

	data.resp.response_code = response_code;
	req.respond();
	return 0;
}

bool HandlerWriteElement::on_init(ln::client* client) {
	this->register_write_element(client, "dude.tf.write_element", "main_group");
	return true;
}

HandlerWriteElement::~HandlerWriteElement() {
	this->unregister_write_element();
}

int HandlerWriteElement::on_write_element(ln::service_request& req,
                                          dude::tf::requests::write_element_t& data) {
	const auto& status = data.req.status;
	id_t id            = utils::to_dude_id(data.req.element_id);

	std::optional<Frame> frame;
	if(is_flag(status, StatusCodes::Frame)) {
		frame = utils::to_dude_frame(data.req.frame);
	}

	std::optional<id_t> parent;
	if(is_flag(status, StatusCodes::Parent)) {
		parent = utils::to_dude_id(data.req.parent);
	}

	const auto& response    = server().on_write_element(id, frame, parent);
	data.resp.response_code = response;
	req.respond();
	return 0;
}

bool HandlerReadElement::on_init(ln::client* client) {
	this->register_read_element(client, "dude.tf.read_element", "main_group");
	return true;
}
HandlerReadElement::~HandlerReadElement() {
	this->unregister_read_element();
}
int HandlerReadElement::on_read_element(ln::service_request& req,
                                        dude::tf::requests::read_element_t& data) {
	const auto& status = data.req.status;
	id_t id            = utils::to_dude_id(data.req.id);

	bool request_frame    = is_flag(status, StatusCodes::Frame);
	bool request_parent   = is_flag(status, StatusCodes::Parent);
	bool request_children = is_flag(status, StatusCodes::Children);

	const auto& [frame, parent, children, response_code] =
			server().on_read_element(id, request_frame, request_parent, request_children);

	data.resp.response_code = response_code;
	if(frame.has_value()) {
		data.resp.frame = utils::to_ln_frame(frame.value());
	}

	if(parent.has_value()) {
		data.resp.parent = utils::to_ln_id(parent.value());
	}

	data.resp.children_len = children.size();
	data.resp.children     = new dude_tf_types_identifier_t[children.size()];
	for(std::size_t i = 0; i < children.size(); ++i) {
		data.resp.children[i] = utils::to_ln_id(children[i]);
	}

	req.respond();
	delete data.resp.children;
	return 0;
}

bool HandlerReadPath::on_init(ln::client* client) {
	this->register_read_path(client, "dude.tf.read_path", "main_group");
	return true;
}

HandlerReadPath::~HandlerReadPath() {
	this->unregister_read_path();
}

int HandlerReadPath::on_read_path(ln::service_request& req, dude::tf::requests::read_path_t& data) {
	const auto start                  = utils::to_dude_id(data.req.start);
	const auto end                    = utils::to_dude_id(data.req.end);
	const auto& [path, response_code] = server().on_read_path(start, end);
	data.resp.response_code           = response_code;
	if(path.has_value()) {
		data.resp.path_frame = utils::to_ln_frame(path.value());
	}
	req.respond();
	return 0;
}
