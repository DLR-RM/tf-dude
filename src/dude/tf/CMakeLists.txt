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

##############################
# Core module
##############################
message(STATUS "Configuring core module")

find_package(Boost REQUIRED COMPONENTS graph)
find_package(Eigen3 REQUIRED)

find_files_by_pattern("core/*.cc" SOURCES_MODULE_CORE)
find_files_by_pattern("core/*.h" HEADERS_MODULE_CORE)

add_library(core ${SOURCES_MODULE_CORE})
target_link_libraries(core PUBLIC Boost::graph Eigen3::Eigen)
install(FILES ${HEADERS_MODULE_CORE} DESTINATION ${INSTALL_INCLUDE_DIR}/dude/tf/core/)
set_install_of_target(core)

##############################
# Communication module
##############################
message(STATUS "Configuring communication module")

find_package(yaml-cpp REQUIRED)

find_files_by_pattern("communication/*.cc" SOURCES_MODULE_COMMUNICATION)
find_files_by_pattern("communication/*.h" HEADERS_MODULE_COMMUNICATION)

add_library(communication ${SOURCES_MODULE_COMMUNICATION})
target_link_libraries(communication PUBLIC core yaml-cpp)
add_dependencies(communication core)
install(FILES ${HEADERS_MODULE_COMMUNICATION} DESTINATION ${INSTALL_INCLUDE_DIR}/dude/tf/communication/)
set_install_of_target(communication)

add_subdirectory(plugins)

##############################
# Final integration including all build plugins
##############################
message(STATUS "Configuring final integration step")

add_library(tf_dude tf_dude_integration.cc)
target_link_libraries(tf_dude PUBLIC communication)
target_include_directories(tf_dude PRIVATE ${GEN_DIR})
add_dependencies(tf_dude communication)
if(BUILD_MODULE_TF_PLUGINS_LN)
    target_link_libraries(tf_dude PUBLIC plugin_ln)
    add_dependencies(tf_dude plugin_ln)
endif ()
if(BUILD_MODULE_TF_PLUGINS_ROS)
    target_link_libraries(tf_dude PUBLIC plugin_ros)
    add_dependencies(tf_dude plugin_ros)
endif ()
install(FILES include.h DESTINATION ${INSTALL_INCLUDE_DIR}/dude/tf)
set_install_of_target(tf_dude)