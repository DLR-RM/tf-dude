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
if (BUILD_MODULE_TF_PLUGINS)
    option(BUILD_MODULE_TF_PLUGINS_LN "Build ln plugin" OFF)
    option(BUILD_MODULE_TF_PLUGINS_ROS "Build ROS plugin" ON)
    option(BUILD_MODULE_TF_PLUGINS_ROS2 "Build ROS2 plugin" OFF)
    set(DEFAULT_PLUGIN "ros" CACHE STRING "Select the default plugin. Must be build")

    if (BUILD_MODULE_TF_PLUGINS_LN)
        message(STATUS "Plugin links_and_nodes")

        # build message definitions
        set(LN_MSG_DEF_DIR "${SHARE_DIR}/ln/msg_defs")
        find_files_in_dir("${LN_MSG_DEF_DIR}" LN_MSG_DEFS)

        add_custom_target(plugin_ln_messages)
        add_custom_command(
                TARGET plugin_ln_messages
                COMMAND ln_generate -f -md_dir ${LN_MSG_DEF_DIR} ${LN_MSG_DEFS} -o ${GEN_DIR}/ln/ln_messages.h
                COMMENT "Generating ln_messages.h out of ${LN_MSG_DEF_DIR}"
        )

        # ln implementation
        find_package(LN REQUIRED)

        find_files_by_pattern("ln/*.cc" SOURCES_MODULE_PLUGIN_LN)
        find_files_by_pattern("ln/*.h" HEADERS_MODULE_PLUGIN_LN)

        add_library(plugin_ln SHARED ${SOURCES_MODULE_PLUGIN_LN})
        target_link_libraries(plugin_ln PUBLIC communication ${LN_LIBRARIES})
        target_include_directories(plugin_ln PUBLIC ${LN_INCLUDE_DIRS})
        target_include_directories(plugin_ln PRIVATE ${GEN_DIR})
        add_dependencies(plugin_ln core communication plugin_ln_messages)

        install(FILES ${HEADERS_MODULE_PLUGIN_LN} DESTINATION ${INSTALL_INCLUDE_DIR}/dude/tf/plugins/ln/)
        set_install_of_target(plugin_ln)

        list(APPEND SUPPORTED_PLUGINS "ln")
    endif ()

    if (BUILD_MODULE_TF_PLUGINS_ROS)
        message(STATUS "Plugin ROS")

        # find ros
        find_package(catkin REQUIRED COMPONENTS roscpp rospy message_generation geometry_msgs)

        # build messages/services
        find_files_in_dir("${SHARE_DIR}/ros/tf_dude/srv" PLUGIN_ROS_SERVICES)
        add_service_files(
                DIRECTORY
                ${SHARE_DIR}/ros/tf_dude/srv
                FILES
                ${PLUGIN_ROS_SERVICES}
        )
        generate_messages(
                DEPENDENCIES
                geometry_msgs
        )

        # ros implementation
        find_files_by_pattern("ros/*.cc" SOURCES_MODULE_PLUGIN_ROS)
        find_files_by_pattern("ros/*.h" HEADERS_MODULE_PLUGIN_ROS)
        add_library(plugin_ros SHARED ${SOURCES_MODULE_PLUGIN_ROS})
        target_link_libraries(plugin_ros PUBLIC ${roscpp_LIBRARIES} communication)
        target_include_directories(plugin_ros PUBLIC ${roscpp_INCLUDE_DIRS})
        target_include_directories(plugin_ros PRIVATE tf_dude_generate_messages)
        add_dependencies(plugin_ros tf_dude_generate_messages communication)

        install(FILES ${HEADERS_MODULE_PLUGIN_ROS} DESTINATION ${INSTALL_INCLUDE_DIR}/dude/tf/plugins/ros/)
        set_install_of_target(plugin_ros)

        list(APPEND SUPPORTED_PLUGINS "ros")
    endif ()

    # check setup
    if(NOT "${DEFAULT_PLUGIN}" IN_LIST SUPPORTED_PLUGINS)
        message(FATAL_ERROR "Selected default plugin is ${DEFAULT_PLUGIN} but it is not build or unknown. Supported are: " ${SUPPORTED_PLUGINS})
    endif ()

    # configure build header
    configure_file(build_options.h.in ${GEN_DIR}/dude/tf/plugins/build_options.h @ONLY)
    install(FILES ${GEN_DIR}/dude/tf/plugins/build_options.h DESTINATION ${INSTALL_INCLUDE_DIR}/dude/tf/plugins/build_options.h)
endif ()
