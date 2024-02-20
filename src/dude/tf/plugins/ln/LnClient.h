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

#ifndef TF_DUDE_LNCLIENT_H
#define TF_DUDE_LNCLIENT_H

#include <dude/tf/communication/Ipc.h>

#include <ln/ln.h>

#include <memory>
#include <string>
#include <vector>

namespace dude::tf::plugins {

class LnClient : public IPCClientBase {
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
	std::shared_ptr<ln::service> get_service(const std::string &service_name);

	/* ln stuff */
	std::string m_ln_manager;
	std::string m_client_name;

	/* pointer to ln client instance */
	std::shared_ptr<ln::client> mp_client;

	static constexpr double TIMEOUT = 0.0;
};
}  // namespace dude::tf::plugins
#endif  // TF_DUDE_LNCLIENT_H
