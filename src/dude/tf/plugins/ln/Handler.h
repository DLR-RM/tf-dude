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

#include <ln/ln.h>
#include <ln/ln_messages.h>

namespace dude::tf {
class IPCServerBase;
}

namespace dude::tf::plugins {

class Handler {
public:
	/**
	 * init function to be called on every handler
	 */
	bool init(ln::client* client, IPCServerBase* p_server);

	/**
	 * specific init for each derived handler
	 * \param client
	 * \return
	 */
	virtual bool on_init(ln::client* client) = 0;

protected:
	/**
	 * Get access to server instance
	 * \return
	 */
	IPCServerBase& server() const {
		return *mp_server;
	}

private:
	IPCServerBase* mp_server;
};

class HandlerWriteElement : public Handler, public dude::tf::requests::write_element_base {
public:
	bool on_init(ln::client* client) override;
	virtual ~HandlerWriteElement();
	int on_write_element(ln::service_request& req,
	                     dude::tf::requests::write_element_t& data) override;
};

class HandlerReadElement : public Handler, public dude::tf::requests::read_element_base {
public:
	bool on_init(ln::client* client) override;
	virtual ~HandlerReadElement();
	int on_read_element(ln::service_request& req, dude::tf::requests::read_element_t& data) override;
};

class HandlerReadPath : public Handler, public dude::tf::requests::read_path_base {
public:
	bool on_init(ln::client* client) override;
	virtual ~HandlerReadPath();
	int on_read_path(ln::service_request& req, dude::tf::requests::read_path_t& data) override;
};

class HandlerRemoveElement : public Handler, public dude::tf::requests::remove_element_base {
public:
	bool on_init(ln::client* client) override;
	virtual ~HandlerRemoveElement();
	int on_remove_element(ln::service_request& req,
	                      dude::tf::requests::remove_element_t& data) override;
};

}  // namespace dude::tf::plugins
#endif  // TF_DUDE_HANDLER_H
