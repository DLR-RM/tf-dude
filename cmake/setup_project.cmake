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

#
# additional packages for this project
#
find_package(PkgConfig REQUIRED)

#
# file paths
#
get_filename_component(ROOT_DIR "${CMAKE_CURRENT_LIST_DIR}" PATH)
get_filename_component(SOURCE_DIR "${ROOT_DIR}/src" ABSOLUTE)
get_filename_component(BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR} ABSOLUTE)
get_filename_component(TEST_DIR "${ROOT_DIR}/test" ABSOLUTE)
get_filename_component(EXT_DIR "${ROOT_DIR}/ext" ABSOLUTE)
get_filename_component(APP_DIR "${ROOT_DIR}/app" ABSOLUTE)
get_filename_component(PY_DIR "${ROOT_DIR}/python" ABSOLUTE)
get_filename_component(SHARE_DIR "${ROOT_DIR}/share" ABSOLUTE)
get_filename_component(EXAMPLES_DIR "${ROOT_DIR}/examples/cpp" ABSOLUTE)
get_filename_component(GEN_DIR ${BUILD_DIR}/gen ABSOLUTE)

#
# install section
#
set(INSTALL_LIB_DIR "lib" CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR "bin" CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_DIR "include" CACHE PATH "Installation directory for header files")

#
# setup version
#
if(NOT DEFINED VERSION_STRING)
    set(tf_dude_MAJOR_VERSION 0)
    set(tf_dude_MINOR_VERSION 0)
    set(tf_dude_PATCH_VERSION 0)
    set(tf_dude_VERSION ${tf_dude_MAJOR_VERSION}.${tf_dude_MINOR_VERSION}.${tf_dude_PATCH_VERSION})
    message(STATUS "setup default version ${tf_dude_VERSION}")

else()
    string(REPLACE "." ";" VERSION_LIST ${VERSION_STRING})

    list(GET VERSION_LIST 0 tf_dude_MAJOR_VERSION)
    list(GET VERSION_LIST 1 tf_dude_MINOR_VERSION)
    list(GET VERSION_LIST 2 tf_dude_PATCH_VERSION)
    set(tf_dude_VERSION ${tf_dude_MAJOR_VERSION}.${tf_dude_MINOR_VERSION}.${tf_dude_PATCH_VERSION})
    message(STATUS "setup user version ${tf_dude_VERSION}")

endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ROOT_DIR}/cmake/modules)