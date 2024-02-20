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

#ifndef TF_DUDE_LNSERVER_H
#define TF_DUDE_LNSERVER_H

#include <dude/tf/communication/Ipc.h>

#include <ln/ln.h>

#include <memory>
#include <string>
#include <vector>

namespace dude::tf::plugins {

class Handler;

class LnServer : public IPCServerBase {
protected:
	bool on_configuration(const YAML::Node &config) override;
	bool on_init() override;
	bool on_start() override;
	bool on_stop() override;

private:
	/* ln client info */
	std::string m_ln_manager;
	std::string m_client_name;

	/* pointer to ln client instance */
	std::shared_ptr<ln::client> mp_client;

	/* list of service handler */
	std::vector<std::shared_ptr<Handler>> m_handler;
};
}  // namespace dude::tf::plugins
#endif  // TF_DUDE_LNSERVER_H
