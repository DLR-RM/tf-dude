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
// Burkhard, Lukas
//
// *********************************************************

#include "Frame.h"

#include <manif/manif.h>

using namespace dude::tf;

Frame Frame::compose(const Frame& other) const {
	// get rotations
	Frame::RotationQuaternion_t quat_A = this->rotation();
	Frame::RotationQuaternion_t quat_B = other.rotation();

	// normalize quaternions for safe computation
	quat_A.normalize();
	quat_B.normalize();

	// convert to internal Lie Algebra
	manif::SE3<Frame::MatrixType_t> A{this->translation(), quat_A};
	manif::SE3<Frame::MatrixType_t> B{other.translation(), quat_B};

	// composition of mean poses
	const auto& composition_se3 = A.compose(B);

	// composition of covariances: cov_A is moved to the reference frame of B and there added to cov_B
	const Frame::CovMatrix_t composition_cov =
			B.inverse().adj() * this->cov() * B.inverse().adj().transpose() + other.cov();

	// return as Frame
	return Frame{composition_se3.transform(), composition_cov};
}

Frame Frame::inverse() const {
	// get rotation and normalize it
	Frame::RotationQuaternion_t quat_A = this->rotation();
	quat_A.normalize();

	// convert to internal Lie Algebra
	manif::SE3<Frame::MatrixType_t> A{this->translation(), quat_A};

	// inverse
	const auto& inverse_se3 = A.inverse();

	// Cov A is moved to the other side of the relative transform
	const auto& inverse_cov = A.adj() * this->cov() * A.adj().transpose();

	// return as Frame
	return Frame{inverse_se3.transform(), inverse_cov};
}