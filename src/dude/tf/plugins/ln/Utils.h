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
#include <dude/tf/core/Types.h>

#include <ln/ln_messages.h>

#include <string>

namespace dude::tf::plugins::utils {

inline dude_tf_types_identifier_t to_ln_id(const dude::tf::id_t& id) {
	dude_tf_types_identifier_t ln_id{};
	ln_id.id     = const_cast<char*>(id.c_str());
	ln_id.id_len = id.size();
	return ln_id;
}

inline dude::tf::id_t to_dude_id(const dude_tf_types_identifier_t& id) {
	return id_t(id.id, id.id_len);
}

inline dude_tf_types_translation_t to_ln_translation(
		const dude::tf::Frame::TranslationVector_t& t) {
	dude_tf_types_translation_t ln_t{};
	ln_t.x = t.x();
	ln_t.y = t.y();
	ln_t.z = t.z();
	return ln_t;
}

inline dude::tf::Frame::TranslationVector_t to_dude_translation(
		const dude_tf_types_translation_t& t) {
	return {t.x, t.y, t.z};
}

inline dude_tf_types_rotation_t to_ln_rotation(const dude::tf::Frame::RotationQuaternion_t& R) {
	dude_tf_types_rotation_t ln_R{};
	ln_R.qw = R.w();
	ln_R.qx = R.x();
	ln_R.qy = R.y();
	ln_R.qz = R.z();
	return ln_R;
}

inline dude::tf::Frame::RotationQuaternion_t to_dude_rotation(const dude_tf_types_rotation_t& R) {
	return {R.qw, R.qx, R.qy, R.qz};
}

inline dude_tf_types_covariance_t to_ln_covariance(const dude::tf::Frame::CovMatrix_t& Cov) {
	dude_tf_types_covariance_t ln_Cov{};
	for(long idx = 0; idx < Cov.rows() * Cov.cols(); ++idx) {
		ln_Cov.value[idx] = Cov(idx);
	}
	return ln_Cov;
}

inline dude::tf::Frame::CovMatrix_t to_dude_covariance(const dude_tf_types_covariance_t& Cov) {
	dude::tf::Frame::CovMatrix_t dude_Cov = dude::tf::Frame::CovMatrix_t::Zero();
	for(long idx = 0; idx < dude_Cov.rows() * dude_Cov.cols(); ++idx) {
		dude_Cov(idx) = Cov.value[idx];
	}
	return dude_Cov;
}

inline dude_tf_types_frame_t to_ln_frame(const dude::tf::Frame& frame) {
	dude_tf_types_frame_t ln_frame{};
	ln_frame.T   = to_ln_translation(frame.translation());
	ln_frame.R   = to_ln_rotation(frame.rotation());
	ln_frame.cov = to_ln_covariance(frame.cov());
	return ln_frame;
}

inline dude::tf::Frame to_dude_frame(const dude_tf_types_frame_t& frame) {
	return Frame(to_dude_translation(frame.T), to_dude_rotation(frame.R),
	             to_dude_covariance(frame.cov));
}

}  // namespace dude::tf::plugins::utils

#endif  // TF_DUDE_UTILS_H
