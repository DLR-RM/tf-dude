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
if (BUILD_EXAMPLES)
    message(STATUS "Configuring examples")


    add_executable(example_client_basic client_basic.cc)
    target_link_libraries(example_client_basic tf_dude)
    add_dependencies(example_client_basic tf_dude)
endif ()