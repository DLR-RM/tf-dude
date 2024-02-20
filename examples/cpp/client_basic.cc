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

#include <iostream>

int main(int argc, char** argv) {
	// create a client for tf_dude
	using namespace dude::tf;
	auto client = create_client();
	client.start();

	// populate tree
	client.add_element("root", Frame(Frame::TranslationVector_t{10, 0, 0}));
	client.add_element("robot", Frame(Frame::TranslationVector_t{1, 0, 0}), "root");
	client.add_element("camera", Frame(Frame::TranslationVector_t{0, 0, 1.5}), "robot");
	client.add_element("imu", Frame(Frame::TranslationVector_t{0, 0.1, 1.5}), "robot");

	// get some info
	auto roots = client.get_roots();
	std::cout << "Roots:" << std::endl;
	for(const auto& root: roots) {
		std::cout << " -- " << root << " ";
		auto children = client.get_children(root);
		std::cout << "(Children: " << std::to_string(children.size()) << ")" << std::endl;
	}
	std::cout << std::endl;

	// get the current frame of the robot
	auto frame_robot = client.get_element("robot");
	std::cout << "Robot is at position " << frame_robot.translation().transpose() << std::endl;

	// get info about the children
	auto frame_camera = client.get_path("robot", "camera");
	std::cout << "The relative offset from robot to camera is "
						<< frame_camera.translation().transpose() << std::endl;
	auto frame_cam_imu = client.get_path("camera", "imu");
	std::cout << "The relative offset from camera to imu is "
						<< frame_cam_imu.translation().transpose() << std::endl;

	// cleanly stop client before exiting
	client.stop(true);

	return 0;
}