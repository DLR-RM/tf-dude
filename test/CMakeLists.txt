# *********************************************************
# tf_dude
# (c) 2022-2024
# Deutsches Zentrum fuer Luft- und Raumfahrt e.V.
# Institute fuer Robotik und Mechatronik
#
# German Aerospace Center
# Institute for Robotics and Mechatronics
#
# This file is part of the tf_dude repository and is provided as is.
# The repository's license does apply.
# 
# *********************************************************
#
# Authors:
# Sewtz, Marco
#
# *********************************************************
if (BUILD_TESTS)
    dump_cmake_variables(GTEST)
    find_package(GTest REQUIRED)

    add_executable(test_core test_core.cc)
    target_link_libraries(test_core core GTest::gtest_main)
    add_dependencies(test_core core)
endif ()