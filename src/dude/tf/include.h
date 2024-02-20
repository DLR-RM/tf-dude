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

#ifndef TF_DUDE_INCLUDE_H
#define TF_DUDE_INCLUDE_H

#include <dude/tf/communication/Client.h>
#include <dude/tf/communication/Server.h>

#include <optional>
#include <string>
#include <vector>

namespace dude::tf {

/**
 * Return a list of all supported plugins
 * \return
 */
[[maybe_unused]] std::vector<std::string> get_supported_plugins();

/**
 * Get the default plugin that will be used
 * \return
 */
[[maybe_unused]] std::string get_default_plugin();

/**
 * Create a tf_dude client. Will use the selected plugin
 * \param plugin_name Name of the plugin. If not set, will
 *                    use the default plugin
 * \return Created client object
 */
[[maybe_unused]] Client create_client(const std::optional<std::string>& plugin_name = std::nullopt);

/**
 * Create a tf_dude server. Will use the selected plugin
 * \param plugin_name Name of the plugin. If not set, will
 *                    use the default plugin
 * \return Created server object
 */
[[maybe_unused]] Server create_server(const std::optional<std::string>& plugin_name = std::nullopt);

}  // namespace dude::tf

#endif  // TF_DUDE_INCLUDE_H
