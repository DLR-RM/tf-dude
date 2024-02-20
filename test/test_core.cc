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

#include <gtest/gtest.h>

#include <dude/tf/core/Tree.h>

TEST(TEST_MODULE_CORE, CREATE_FRAME) {
	using namespace dude::tf;

	// create an empty frame
	auto f0 = Frame();
	EXPECT_TRUE(f0.pose().isApprox(Frame::PoseMatrix_t::Identity()));
	EXPECT_TRUE(f0.cov().isApprox(Frame::CovMatrix_t::Zero()));

	// create with initial pose matrix
	Frame::PoseMatrix_t init_pose_matrix = Frame::PoseMatrix_t::Identity();
	init_pose_matrix.block<3, 1>(0, 3)   = Frame::TranslationVector_t{1, 2, 3};
	auto f1                              = Frame(init_pose_matrix);
	EXPECT_TRUE(f1.pose().isApprox(init_pose_matrix));
	EXPECT_TRUE(f1.cov().isApprox(Frame::CovMatrix_t::Zero()));

	// create with initial cov matrix
	Frame::CovMatrix_t init_cov_matrix = Frame::CovMatrix_t::Zero();
	init_cov_matrix(0, 0)              = 1;
	init_cov_matrix(1, 1)              = 2;
	init_cov_matrix(2, 2)              = 3;
	auto f2                            = Frame(init_pose_matrix, init_cov_matrix);
	EXPECT_TRUE(f2.pose().isApprox(init_pose_matrix));
	EXPECT_TRUE(f2.cov().isApprox(init_cov_matrix));
}

TEST(TEST_MODULE_CORE, FRAME_BASIC_OPERATION) {
	using namespace dude::tf;

	// create an empty frame
	auto f0 = Frame();
	auto f1 = Frame();

	// translate
	Frame::PoseMatrix_t altered_pose_matrix = Frame::PoseMatrix_t::Identity();
	altered_pose_matrix.block<3, 1>(0, 3)   = Frame::TranslationVector_t{1, 2, 3};
	f1.set_translation({1, 2, 3});
	EXPECT_TRUE(f1.pose().isApprox(altered_pose_matrix)) << f1.pose() << std::endl;
	EXPECT_TRUE(f1.cov().isApprox(Frame::CovMatrix_t::Zero())) << f1.cov() << std::endl;

	// rotate
	Frame::RotationQuaternion_t altered_rotation =
			Frame::RotationQuaternion_t{0.7071067811865475, 0.7071067811865475, 0., 0.};
	f1.set_rotation(altered_rotation);
	altered_pose_matrix.block<3, 3>(0, 0) = altered_rotation.matrix();
	EXPECT_TRUE(f1.pose().isApprox(altered_pose_matrix)) << f1.pose() << std::endl
																											 << altered_pose_matrix << std::endl;
	EXPECT_TRUE(f1.cov().isApprox(Frame::CovMatrix_t::Zero())) << f1.cov() << std::endl;

	// compose
	auto f2 = f0.compose(f1);

	EXPECT_TRUE(f2.pose().isApprox(f1.pose())) << f2.pose() << std::endl << f1.pose() << std::endl;
	EXPECT_TRUE(f2.cov().isApprox(f1.cov())) << f1.cov() << std::endl;
}

TEST(TEST_MODULE_CORE, FRAME_COVARIANCE) {
	using namespace dude::tf;

	// Test the correctness of the covariance computation using a numerical example

	// poses
	Frame::PoseMatrix_t pose_A = Frame::PoseMatrix_t::Identity();
	pose_A(0, 3)               = 14.;
	Frame::PoseMatrix_t pose_B = Frame::PoseMatrix_t::Identity();
	pose_B(0, 0)               = 0.8776;
	pose_B(0, 1)               = -0.4794;
	pose_B(1, 0)               = 0.4794;
	pose_B(1, 1)               = 0.8776;
	pose_B(0, 3)               = 95.8851;
	pose_B(1, 3)               = 24.4835;

	// uncertainties
	Eigen::Matrix<double, 6, 6> cov_A, cov_B, cov_result;
	cov_B.setZero();
	cov_A.setZero();
	cov_A(5, 5) = 0.01;
	cov_result << 5.992959784318268, 23.473359525357974, 0.0, 0.0, 0.0, 0.24480522429715973,
			23.473359525357974, 91.94098195828174, 0.0, 0.0, 0.0, 0.9588586024971656, 0.0, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.24480522429715973,
			0.9588586024971656, 0.0, 0.0, 0.0, 0.02;

	Frame A      = Frame(pose_A, cov_A);
	Frame B      = Frame(pose_B, cov_B);
	Frame result = Frame();
	result.set_translation({109.8851, 24.4835, 0.0});
	result.set_rotation({0.9689162282045662, 0.0, 0.0, 0.24738905133784092});

	// compose
	Frame composition = A.compose(B);

	// Check results:
	EXPECT_TRUE(composition.pose().isApprox(result.pose(), 1e-7)) << composition.pose() << "\n---\n"
																																<< result.pose() << std::endl;
	EXPECT_TRUE(composition.cov().isApprox(cov_result, 5e-3)) << composition.cov() << "\n---\n"
																														<< cov_result << std::endl;
}

TEST(TEST_MODULE_CORE, FRAME_ADVANCED_OPERATION) {
	using namespace dude::tf;

	// create an empty frame
	auto f0 = Frame();
	auto f1 = Frame();

	// initial covariance
	Frame::CovMatrix_t init_cov = Frame::CovMatrix_t::Zero();
	init_cov(0, 0)              = 1;
	init_cov(1, 1)              = 2;
	f0.set_covariance(init_cov);
	f1.set_covariance(init_cov);

	// compose
	f1.set_translation({1, 2, 3});
	auto f2 = f0.compose(f1);

	EXPECT_TRUE(f2.pose().isApprox(f1.pose())) << f2.pose() << std::endl << f1.pose() << std::endl;
	EXPECT_TRUE(f2.cov().isApprox(2 * f1.cov())) << f2.cov() << std::endl;
}

TEST(TEST_MODULE_CORE, TREE_CREATE) {
	using namespace dude::tf;

	// create a new tree
	Tree tree;

	EXPECT_TRUE(tree.empty());
	EXPECT_EQ(tree.size(), 0);
}

TEST(TEST_MODULE_CORE, TREE_BASIC_OPERATION) {
	using namespace dude::tf;

	// create a new tree
	Tree tree;
	auto f_root = Frame();
	auto f_A1   = Frame(Frame::TranslationVector_t{1, 2, 3}, Frame::RotationQuaternion_t{1, 0, 0, 0});
	auto f_A2 = Frame(Frame::TranslationVector_t{10, 0, 0}, Frame::RotationQuaternion_t{1, 0, 0, 0});
	auto f_B1 = Frame(Frame::TranslationVector_t{1, 0, 0}, Frame::RotationQuaternion_t{0, 0, 0, 1});

	// add root element
	tree.add_element("root", f_root);
	EXPECT_FALSE(tree.empty());
	EXPECT_EQ(tree.size(), 1);
	EXPECT_TRUE(tree.exists("root"));
	EXPECT_FALSE(tree.exists("wrong"));
	EXPECT_TRUE(tree.get("root").pose().isApprox(f_root.pose()));
	EXPECT_TRUE(tree.get("root").cov().isApprox(f_root.cov()));

	// add more elements
	tree.add_element("A1", f_A1, "root");
	tree.add_element("A2", f_A2, "A1");
	tree.add_element("B1", f_B1, "root");
	EXPECT_FALSE(tree.empty());
	EXPECT_EQ(tree.size(), 4);
	EXPECT_TRUE(tree.exists("root"));
	EXPECT_TRUE(tree.exists("A1"));
	EXPECT_TRUE(tree.exists("A2"));
	EXPECT_TRUE(tree.exists("B1"));
	;
	EXPECT_TRUE(tree.get("A1").pose().isApprox(f_A1.pose()));
	EXPECT_TRUE(tree.get("A1").cov().isApprox(f_A1.cov()));
	EXPECT_EQ(tree.get_all_elements().size(), tree.size());
	EXPECT_EQ(tree.get_all_connections().size(), 6);
	EXPECT_EQ(tree.get_all_children("root").size(), 2);
	EXPECT_EQ(tree.get_all_children("A1").size(), 1);
	EXPECT_EQ(tree.get_all_children("B1").size(), 0);

	// get path
	EXPECT_EQ(tree.find_path("root", "A2").size(), 2);
	EXPECT_TRUE(tree.get_composed_transform("root", "A1")->pose().isApprox(f_A1.pose()));
	EXPECT_TRUE(tree.get_composed_transform("A1", "root")->pose().isApprox(f_A1.pose().inverse()));
	EXPECT_TRUE(tree.get_composed_transform("A2", "root")
	                ->pose()
	                .isApprox((f_A1.pose() * f_A2.pose()).inverse()));

	// move A branch to B
	tree.move_element("A1", "B1");
	EXPECT_FALSE(tree.empty());
	EXPECT_EQ(tree.size(), 4);
	EXPECT_TRUE(tree.exists("root"));
	EXPECT_TRUE(tree.exists("A1"));
	EXPECT_TRUE(tree.exists("A2"));
	EXPECT_TRUE(tree.exists("B1"));
	;
	EXPECT_TRUE(tree.get("A1").pose().isApprox(f_A1.pose()));
	EXPECT_TRUE(tree.get("A1").cov().isApprox(f_A1.cov()));
	EXPECT_EQ(tree.get_all_elements().size(), tree.size());
	EXPECT_EQ(tree.get_all_connections().size(), 6);
	EXPECT_EQ(tree.get_all_children("root").size(), 1);
	EXPECT_EQ(tree.get_all_children("A1").size(), 1);
	EXPECT_EQ(tree.get_all_children("B1").size(), 1);

	// get path
	EXPECT_EQ(tree.find_path("root", "A2").size(), 3);
	EXPECT_EQ(tree.find_path("A1", "A2").size(), 1);
	EXPECT_TRUE(tree.get_composed_transform("root", "A2")
	                ->pose()
	                .isApprox(f_B1.pose() * f_A1.pose() * f_A2.pose()));
}