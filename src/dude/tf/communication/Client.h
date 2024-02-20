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

#ifndef TF_DUDE_CLIENT_H
#define TF_DUDE_CLIENT_H

#include "Codes.h"
#include <dude/tf/core/Frame.h>
#include <dude/tf/core/Types.h>

#include <yaml-cpp/yaml.h>

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace dude::tf {

// forward declaration
class IPCClientBase;

class Client {
public:
	/**
	 * Create a new client instance based on specific ipc implementation
	 * \param ipc ipc implementation
	 */
	explicit Client(std::shared_ptr<IPCClientBase> ipc);

	/**
	 * Destructor. Stop client if necessary.
	 */
	~Client();

	/**
	 * Configure client
	 * \param config yaml configuration
	 * \return True if successful, False otherwise
	 */
	bool configure(const YAML::Node& config);

	/**
	 * Configure client
	 * \param config_path path to a yaml-compliant file
	 * \return True if successful, False otherwise
	 */
	bool configure(const std::string& config_path);

	/**
	 * Configure with default settings;
	 * \return
	 */
	bool configure_default();

	/**
	 * Start client TODO: this might be implicit
	 * \return
	 */
	bool start();

	/**
	 * Stop client
	 * \param wait if True wait for client to stop
	 */
	void stop(bool wait = false);

	/**
	 * Return whether the client is running
	 * \return
	 */
	[[nodiscard]] bool is_running() const;

	//
	//
	//

	bool add_element(const id_t& id, const Frame& frame, const id_t& parent = NO_ID);

	bool element_exists(const id_t& id);

	Frame get_element(const id_t& id);

	bool update_element(const id_t& id, const Frame& new_frame);

	bool move_element(const id_t& id, const id_t& new_parent);

	bool remove_element(const id_t& id);

	std::vector<id_t> get_roots();

	std::vector<id_t> get_children(const id_t& parent);

	Frame get_path(const id_t& start, const id_t& end);

	bool path_exists(const id_t& start, const id_t& end);

private:
	/* pointer to IPC specific implementation */
	std::shared_ptr<IPCClientBase> mp_ipc;

	/* whether this client has been configured or not */
	bool m_is_configured = false;
};

}  // namespace dude::tf
#endif  // TF_DUDE_CLIENT_H
