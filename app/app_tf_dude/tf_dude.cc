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

#include <dude/tf/include.h>

#include <tclap/CmdLine.h>

#include <atomic>
#include <chrono>
#include <string>
#include <thread>
#include <csignal>
#include <optional>
#include <iostream>

using namespace dude::tf;
using namespace std::chrono_literals;

std::atomic_bool stop_flag{false};
void sig_handler(int sig) {
	stop_flag = true;
}

int main(int argc, char** argv) {

	auto loaded_ipc_plugins = get_supported_plugins();
	TCLAP::ValuesConstraint<std::string> ipc_constraints(loaded_ipc_plugins);

	// CLI (TODO: add feedback loop from IPC implementation here)
	TCLAP::CmdLine cmd("tf_dude -Central transformation service provider.\n\nMarco Sewtz, DLR-RMC");
	TCLAP::ValueArg<std::string> arg_ipc("", "ipc",
	                                     "Type of the inter-process communication implementation.",
	                                     false, "", &ipc_constraints);
	TCLAP::ValueArg<std::string> arg_config("c", "config", "A yaml-compliant configuration file", false,
	                                            "", "Path to file");
	try {
		cmd.add(arg_ipc);
		cmd.add(arg_config);
		cmd.parse(argc, argv);
	} catch(TCLAP::CmdLineParseException& e) {
		std::cerr << e.what() << std::endl;
		return EXIT_FAILURE;
	}

	// register signals
	std::signal(SIGINT, &sig_handler);
	std::signal(SIGTERM, &sig_handler);

	// create server
	std::optional<std::string> plugin_name;
	if(arg_ipc.isSet()){
		plugin_name = arg_ipc.getValue();
	}
	auto server = create_server(plugin_name);
	if(arg_config.isSet()){
		server.configure(arg_config.getValue());
	}

	// run
	if(not server.start()) {
		std::cerr << "Could not start server" << std::endl;
		return EXIT_FAILURE;
	}
	std::cout << "running..." << std::endl;
	while(not stop_flag) {
		std::this_thread::sleep_for(100ms);
	}
	std::cout << "stopping..." << std::endl;
	server.stop();

	return EXIT_SUCCESS;
}