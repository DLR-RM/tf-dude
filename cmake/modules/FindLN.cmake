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

# check for LN variables
if($ENV{LN_ROOT})
    message(STATUS "Found LN_ROOT environment variable")
elseif(CONAN_LIBLINKS_AND_NODES_ROOT)
    message(STATUS "Found a conan variable for LN")
    set(LN_ROOT ${CONAN_LIBLINKS_AND_NODES_ROOT})
elseif (ENV{CONAN_LIBLINKS_AND_NODES_ROOT})
    message(STATUS "Found a conan variable for LN")
    set(LN_ROOT ${CONAN_LIBLINKS_AND_NODES_ROOT})
endif()

# maybe add other ways of finding LN root

# continue if LN was found
if(LN_ROOT)
    message(STATUS "LN root is set to " ${LN_ROOT})

    set(LN_INCLUDE_DIRS ${LN_ROOT}/include)
    set(LN_LIBRARIES ${LN_ROOT}/lib/libln.so)
else()
    message(FATAL_ERROR "Could not find LN. Add a conan environment, set LN_ROOT or add the toolchain file.")
endif()