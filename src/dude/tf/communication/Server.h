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

#ifndef TF_DUDE_SERVER_H
#define TF_DUDE_SERVER_H

#include "Codes.h"
#include <dude/tf/core/Tree.h>

#include <yaml-cpp/yaml.h>

#include <atomic>
#include <memory>
#include <optional>
#include <string>

namespace dude::tf {

// forward declaration
class IPCServerBase;

class Server {
	friend IPCServerBase;

public:
	/**
	 * Create a service instance based on specific IPC implementation
	 * \param ipc IPC implementation
	 */
	explicit Server(std::shared_ptr<IPCServerBase> ipc);

	/**
	 * Configure server using a YAML Node
	 * \param config
	 * \return
	 */
	bool configure(const YAML::Node& config);

	/**
	 * Configure server using a yaml-compliant file
	 * \param config_path
	 * \return
	 */
	bool configure(const std::string& config_path);

	/**
	 * Configure with default settings;
	 * \return
	 */
	bool configure_default();

	/**
	 * Start the server
	 */
	bool start();

	/**
	 * Stop the server
	 * \param wait If true, wait until server is stopped
	 */
	void stop(bool wait = false);

	/**
	 * Return whether the server is running
	 * \return True if running, False otherwise
	 */
	bool is_running() const;

private:
	Status_t on_write_element(const id_t& id, const std::optional<Frame>& frame,
	                          const std::optional<id_t>& parent);

	std::tuple<std::optional<Frame>, std::optional<dude::tf::id_t>, std::vector<dude::tf::id_t>,
	           Status_t>
			on_read_element(const id_t& id, bool request_frame, bool request_parent,
	                    bool request_children);

	std::tuple<std::optional<Frame>, Status_t> on_read_path(const id_t& start, const id_t& end);

	Status_t on_remove_element(const id_t& id);

	/* pointer to the specific IPC implementation */
	std::shared_ptr<IPCServerBase> mp_ipc;

	/* the data tree */
	Tree m_tree;

	/* whether this server has been configured or not */
	bool m_is_configured = false;
};

}  // namespace dude::tf
#endif  // TF_DUDE_SERVER_H
