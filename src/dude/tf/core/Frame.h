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

#ifndef TF_DUDE_FRAME_H
#define TF_DUDE_FRAME_H

#include <Eigen/Eigen>

#include <iostream>
#include <utility>

namespace dude::tf {

class Frame {
public:
	using MatrixType_t         = double;
	using PoseMatrix_t         = Eigen::Matrix<MatrixType_t, 4, 4>;
	using CovMatrix_t          = Eigen::Matrix<MatrixType_t, 6, 6>;
	using TranslationVector_t  = Eigen::Matrix<MatrixType_t, 3, 1>;
	using RotationQuaternion_t = Eigen::Quaternion<MatrixType_t>;
	using RotationMatrix_t     = Eigen::Matrix<MatrixType_t, 3, 3>;

	// ****************************************************
	// Constructors
	// ****************************************************
	/**
	 * \brief C'Tor creating an new frame with initial
	 *        pose and covariance information
	 *
	 * \param pose 4x4 matrix initial pose
	 * \param cov 6x6 matrix initial covariance
	 */
	explicit Frame(PoseMatrix_t pose = PoseMatrix_t::Identity(),
	               CovMatrix_t cov   = CovMatrix_t::Zero()) :
			m_pose(std::move(pose)), m_covariance(std::move(cov)) {}

	explicit Frame(const TranslationVector_t& translation,
	               const RotationQuaternion_t& rotation = RotationQuaternion_t::Identity(),
	               const CovMatrix_t& cov               = CovMatrix_t::Zero()) {
		set_translation(translation);
		set_rotation(rotation);
		set_covariance(cov);
	}

	explicit Frame(const TranslationVector_t& translation, const std::array<MatrixType_t, 4> rotation,
	               const CovMatrix_t& cov = CovMatrix_t::Zero()) {
		set_translation(translation);
		set_rotation({rotation[0], rotation[1], rotation[2], rotation[3]});
		set_covariance(cov);
	}

	// ****************************************************
	// Access to data fields
	// ****************************************************

	/**
	 * \brief Get access to pose data
	 * \return Reference to pose data
	 */
	[[nodiscard]] PoseMatrix_t pose() {
		return m_pose;
	}
	[[nodiscard]] PoseMatrix_t pose() const {
		return m_pose;
	}

	/**
	 * \brief Set pose to new pose
	 * \param pose new pose data
	 */
	void set_pose(const PoseMatrix_t& pose) {
		m_pose = pose;
	}

	/**
	 * \brief Set new translation
	 * \param translation new translation
	 */
	void set_translation(const TranslationVector_t& translation) {
		m_pose.block<3, 1>(0, 3) = translation;
	}

	/**
	 * Get the translation as a 3D vector
	 * \return 3D translation vector
	 */
	[[nodiscard]] TranslationVector_t translation() const {
		return m_pose.block<3, 1>(0, 3);
	}

	/**
	 * Get the rotation as a quaternion
	 * \return rotation quaternion
	 */
	[[nodiscard]] RotationQuaternion_t rotation() const {
		return RotationQuaternion_t{m_pose.block<3, 3>(0, 0)};
	}

	/**
	 * \brief Set new rotation
	 * \param rotation new rotation
	 */
	void set_rotation(const RotationQuaternion_t& rotation) {
		m_pose.block<3, 3>(0, 0) = rotation.matrix();
	}

	/**
	 * \brief Get access to covariance data
	 * \return Reference to covariance data
	 */
	CovMatrix_t cov() {
		return m_covariance;
	}
	[[nodiscard]] CovMatrix_t cov() const {
		return m_covariance;
	}

	/**
	 * \brief Set covariance to new covariance data
	 * \param cov new covariance data
	 */
	void set_covariance(const CovMatrix_t& cov) {
		m_covariance = cov;
	}

	// ****************************************************
	// Operators
	// ****************************************************

	/**
	 * \brief The compose operation adds another frame object
	 *        to the this frame and will create the composition
	 *        of both
	 * \param other Frame object to be "added" to this frame
	 * \return Composition of both frames
	 */
	[[nodiscard]] virtual Frame compose(const Frame& other) const;

	/**
	 * \brief The inverse operation will return the inverse
	 *        mathematical object corresponding to this frame
	 * \return Inverse frame
	 */
	[[nodiscard]] virtual Frame inverse() const;

private:
	PoseMatrix_t m_pose{PoseMatrix_t::Identity()};
	CovMatrix_t m_covariance{CovMatrix_t::Zero()};
};

}  // namespace dude::tf

#endif  // TF_DUDE_FRAME_H
