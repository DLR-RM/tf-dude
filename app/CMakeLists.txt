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
if (BUILD_APPS)
    message(STATUS "Configuring apps")

    pkg_check_modules(tclap REQUIRED IMPORTED_TARGET tclap)
    find_package(yaml-cpp REQUIRED)

    find_files_by_pattern("app_tf_dude/*.cc" SOURCES_APP_TF_DUDE)

    add_executable(tf_dude_server ${SOURCES_APP_TF_DUDE})
    target_link_libraries(tf_dude_server PUBLIC PkgConfig::tclap yaml-cpp tf_dude)
    add_dependencies(tf_dude_server tf_dude)
    set_install_of_target(tf_dude_server)
endif ()