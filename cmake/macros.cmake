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
# List all variables
# https://stackoverflow.com/questions/9298278/cmake-print-out-all-accessible-variables-in-a-script
#
function(dump_cmake_variables)
    get_cmake_property(_variableNames VARIABLES)
    list (SORT _variableNames)
    foreach (_variableName ${_variableNames})
        if (ARGV0)
            unset(MATCHED)
            string(REGEX MATCH ${ARGV0} MATCHED ${_variableName})
            if (NOT MATCHED)
                continue()
            endif()
        endif()
        message(STATUS "${_variableName}=${${_variableName}}")
    endforeach()
endfunction()

#
# macro for finding files based on a patter
#
macro(find_files_by_pattern pattern out)
    file(GLOB_RECURSE ${out} "${pattern}")
endmacro()

macro(find_files_in_dir dir out)
    # find all files with absolute path
    find_files_by_pattern("${dir}/*" _files)

    # remove dir from paths
    set(_relative_file)
    foreach(_file ${_files})
        string(REPLACE ${dir} "" _file ${_file})
        string(REGEX REPLACE "^\/" "" _file ${_file})
        set(_relative_file ${_relative_file} ${_file})
    endforeach()

    set(${out} ${_relative_file})
endmacro()

macro(find_files_in_dir dir out)
    # find all files with absolute path
    find_files_by_pattern("${dir}/*" _files)

    # remove dir from paths
    set(_relative_file)
    foreach(_file ${_files})
        string(REPLACE ${dir} "" _file ${_file})
        string(REGEX REPLACE "^\/" "" _file ${_file})
        set(_relative_file ${_relative_file} ${_file})
    endforeach()

    set(${out} ${_relative_file})
endmacro()

#
# macro for setting the install target
#
macro(set_install_of_target name)
    # install target
    install(TARGETS ${name}
            EXPORT DUDE_TARGETS
            RUNTIME DESTINATION ${INSTALL_BIN_DIR}
            ARCHIVE DESTINATION ${INSTALL_LIB_DIR}
            LIBRARY DESTINATION ${INSTALL_LIB_DIR}
            INCLUDES DESTINATION ${INSTALL_INCLUDE_DIR}
    )
endmacro(set_install_of_target)

# Get all properties that cmake supports
# https://stackoverflow.com/a/34292622
if(NOT CMAKE_PROPERTY_LIST)
    execute_process(COMMAND cmake --help-property-list OUTPUT_VARIABLE CMAKE_PROPERTY_LIST)

    # Convert command output into a CMake list
    string(REGEX REPLACE ";" "\\\\;" CMAKE_PROPERTY_LIST "${CMAKE_PROPERTY_LIST}")
    string(REGEX REPLACE "\n" ";" CMAKE_PROPERTY_LIST "${CMAKE_PROPERTY_LIST}")
    list(REMOVE_DUPLICATES CMAKE_PROPERTY_LIST)
endif()

function(print_properties)
    message("CMAKE_PROPERTY_LIST = ${CMAKE_PROPERTY_LIST}")
endfunction()

function(print_target_properties target)
    if(NOT TARGET ${target})
      message(STATUS "There is no target named '${target}'")
      return()
    endif()

    foreach(property ${CMAKE_PROPERTY_LIST})
        string(REPLACE "<CONFIG>" "${CMAKE_BUILD_TYPE}" property ${property})

        # Fix https://stackoverflow.com/questions/32197663/how-can-i-remove-the-the-location-property-may-not-be-read-from-target-error-i
        if(property STREQUAL "LOCATION" OR property MATCHES "^LOCATION_" OR property MATCHES "_LOCATION$")
            continue()
        endif()

        get_property(was_set TARGET ${target} PROPERTY ${property} SET)
        if(was_set)
            get_target_property(value ${target} ${property})
            message("${target} ${property} = ${value}")
        endif()
    endforeach()
endfunction()