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

#ifndef TF_DUDE_UTILS_H
#define TF_DUDE_UTILS_H

#include <dude/tf/core/Frame.h>
#include <geometry_msgs/PoseWithCovariance.h>

using namespace dude::tf;

namespace dude::tf::utils {

inline Frame to_dude_frame(const geometry_msgs::PoseWithCovariance& pose) {
	Frame frame;

	// set pose
	Frame::PoseMatrix_t dude_pose = Frame::PoseMatrix_t::Identity();
	dude_pose.block<3, 1>(0, 3)   = Eigen::Vector3<Frame::MatrixType_t>{
      pose.pose.position.x, pose.pose.position.y, pose.pose.position.z};
	dude_pose.block<3, 3>(0, 0) =
			Eigen::Quaternion<Frame::MatrixType_t>(pose.pose.orientation.w, pose.pose.orientation.x,
	                                           pose.pose.orientation.y, pose.pose.orientation.z)
					.matrix();
	frame.set_pose(dude_pose);

	// set covariance
	for(int i = 0; i < geometry_msgs::PoseWithCovariance::_covariance_type::size(); ++i) {
		int row               = i / 6;
		int col               = i % 6;
		frame.cov()(row, col) = pose.covariance.elems[i];
	}
	return frame;
}

inline geometry_msgs::PoseWithCovariance to_ros_frame(const Frame& frame) {
	geometry_msgs::PoseWithCovariance pose{};

	// set pose
	pose.pose.position.x    = frame.pose()(0, 3);
	pose.pose.position.y    = frame.pose()(1, 3);
	pose.pose.position.z    = frame.pose()(2, 3);
	const auto& q           = frame.rotation();
	pose.pose.orientation.w = q.w();
	pose.pose.orientation.x = q.x();
	pose.pose.orientation.y = q.y();
	pose.pose.orientation.z = q.z();

	// set covariance
	for(int i = 0; i < geometry_msgs::PoseWithCovariance::_covariance_type::size(); ++i) {
		int row                  = i / 6;
		int col                  = i % 6;
		pose.covariance.elems[i] = frame.cov()(row, col);
	}
	return pose;
}

}  // namespace dude::tf::utils

#endif  // TF_DUDE_UTILS_H
