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

if (BUILD_TF_PYTHON_BINDINGS)
    message(STATUS "Configuring Python bindings")

    # download latest pybind11
    include(FetchContent)
    FetchContent_Declare(
            pybind11
            GIT_REPOSITORY https://github.com/pybind/pybind11.git
            GIT_TAG v2.11.1
    )
    message(STATUS "Downloading pybind11")
    FetchContent_MakeAvailable(pybind11)
    FetchContent_GetProperties(pybind11)
    if(NOT pybind11_POPULATED)
        # Fetch the content using previously declared details
        FetchContent_Populate(pybind11)

        # Bring the populated content into the build
        add_subdirectory(${pybind11_SOURCE_DIR} ${pybind11_BINARY_DIR})
    endif()

    pybind11_add_module(py_tf_dude py_tf_dude.cc)
    target_link_libraries(py_tf_dude PRIVATE tf_dude)
    add_dependencies(py_tf_dude tf_dude)
endif ()