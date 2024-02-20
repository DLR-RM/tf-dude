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
cmake_minimum_required(VERSION 3.14)
project(tf_dude_msgs)

# find catkin required packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy message_generation geometry_msgs)

add_service_files(
        DIRECTORY srv
        FILES
        write_element.srv
        read_element.srv
        remove_element.srv
        read_path.srv
)
generate_messages(
        DEPENDENCIES
        geometry_msgs
)