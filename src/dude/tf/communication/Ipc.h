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

#ifndef TF_DUDE_IPC_H
#define TF_DUDE_IPC_H

#include "Client.h"
#include "Codes.h"
#include "Server.h"
#include <dude/tf/core/Types.h>

#include <yaml-cpp/yaml.h>

#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace dude::tf {

class IPCBase {
public:
	/**
	 * Configure the IPC
	 * \param config
	 * \return
	 */
	bool configuration(const YAML::Node& config);

	/**
	 * \brief Open yaml file and parse it for configuring the IPC
	 * \param config_path path to yaml-compliant configuration file
	 * \return True if configuration successful, False otherwise
	 */
	bool configuration(const std::string& config_path) {
		const auto config_node = YAML::LoadFile(config_path);
		return configuration(config_node);
	}

	/**
	 * \brief Perform init step
	 * \return True if init successful, False otherwise
	 */
	bool init();

	/**
	 * \brief Start IPC
	 * \return True if start was successful, False otherwise
	 */
	bool start();

	/**
	 * \brief Stop IPC
	 * \return True if stop successful, False otherwise
	 */
	bool stop();

	/**
	 * \brief Return whether IPC is configured
	 * \return True if configured, False otherwise
	 */
	[[nodiscard]] bool is_configured() const {
		return m_is_configured;
	}

	/**
	 * \brief Return whether the IPC is initialized
	 * \return True if initialized, False otherwise
	 */
	[[nodiscard]] bool is_initialized() const {
		return m_is_initialized;
	}

	/**
	 * \brief Return whether IPC is running
	 * \return True if started, False otherwise
	 */
	[[nodiscard]] bool is_running() const {
		return m_is_running;
	}

protected:
	/**
	 * Specific implementation of the configuration step
	 * \param config
	 * \return
	 */
	virtual bool on_configuration(const YAML::Node& config) = 0;

	/**
	 * Specific implementation of the init step
	 * \return
	 */
	virtual bool on_init() = 0;

	/**
	 * Specific implementation of the start step
	 * \return
	 */
	virtual bool on_start() = 0;

	/**
	 * Specific implementation of the stop step
	 * \return
	 */
	virtual bool on_stop() = 0;

private:
	/* status flags */
	bool m_is_configured{false};
	bool m_is_initialized{false};
	bool m_is_running{false};  // TODO: make implementation specific
};

class IPCServerBase : public IPCBase {
	// give server access to add_server_handle
	friend Server;

public:
	// handler callbacks
	Status_t on_write_element(const id_t& id, const std::optional<Frame>& frame,
	                          const std::optional<id_t>& parent);
	std::tuple<std::optional<Frame>, std::optional<id_t>, std::vector<id_t>, Status_t>
			on_read_element(const id_t& id, bool request_frame, bool request_parent,
	                    bool request_children);

	std::tuple<std::optional<Frame>, Status_t> on_read_path(const id_t& start, const id_t& end);

	bool on_remove_element(const id_t& id);

	virtual ~IPCServerBase() = default;

private:
	void add_server_handle(Server* server) {
		mp_callback_write_element = [server](const id_t& id, const std::optional<Frame>& frame,
		                                     const std::optional<id_t>& parent) {
			return server->on_write_element(id, frame, parent);
		};

		mp_callback_read_element = [server](const id_t& id, bool request_frame, bool request_parent,
		                                    bool request_children) {
			return server->on_read_element(id, request_frame, request_parent, request_children);
		};

		mp_callback_read_path = [server](const id_t& start, const id_t& end) {
			return server->on_read_path(start, end);
		};

		// set callback for remove element
		mp_callback_remove_element = [server](const id_t& id) { return server->on_remove_element(id); };
	}

	// all callbacks in the server class
	std::function<Status_t(const id_t&, const std::optional<Frame>&, const std::optional<id_t>&)>
			mp_callback_write_element;
	std::function<std::tuple<std::optional<Frame>, std::optional<id_t>, std::vector<id_t>, Status_t>(
			const id_t, bool, bool, bool)>
			mp_callback_read_element;
	std::function<std::tuple<std::optional<Frame>, Status_t>(const id_t&, const id_t&)>
			mp_callback_read_path;
	std::function<Status_t(const id_t&)> mp_callback_remove_element;
};

class IPCClientBase : public IPCBase {
	// give client access to this class
	friend Client;

protected:
	virtual Status_t call_write_element(const id_t& id, const std::optional<Frame>& frame,
	                                    const std::optional<id_t>& parent) = 0;

	Status_t call_write_element(const id_t& id, const Frame& new_frame) {
		return call_write_element(id, new_frame, std::nullopt);
	}
	Status_t call_write_element(const id_t& id, const id_t& new_parent) {
		return call_write_element(id, std::nullopt, new_parent);
	}

	virtual std::tuple<std::optional<Frame>, std::optional<id_t>, std::vector<id_t>, Status_t>
			call_read_element(const id_t& id, bool request_frame, bool request_parent,
	                      bool request_children) = 0;

	virtual std::tuple<std::optional<Frame>, Status_t> call_read_path(const id_t& start,
	                                                                  const id_t& end) = 0;

	virtual Status_t call_remove_element(const id_t& id) = 0;

private:
	bool add_element(const id_t& id, const Frame& frame, const id_t& parent);
	bool element_exists(const id_t& id);
	Frame get_element(const id_t& id);
	bool update_element(const id_t& id, const Frame& new_frame);
	bool move_element(const id_t& id, const id_t& new_parent);
	bool remove_element(const id_t& id);
	std::vector<id_t> get_roots();
	std::vector<id_t> get_children(const id_t& id);
	Frame get_path(const id_t& start, const id_t& end);
	bool path_exists(const id_t& start, const id_t& end);
};

}  // namespace dude::tf
#endif  // TF_DUDE_IPC_H
