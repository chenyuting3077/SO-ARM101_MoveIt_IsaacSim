# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_so_arm_teleop_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED so_arm_teleop_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(so_arm_teleop_FOUND FALSE)
  elseif(NOT so_arm_teleop_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(so_arm_teleop_FOUND FALSE)
  endif()
  return()
endif()
set(_so_arm_teleop_CONFIG_INCLUDED TRUE)

# output package information
if(NOT so_arm_teleop_FIND_QUIETLY)
  message(STATUS "Found so_arm_teleop: 0.1.0 (${so_arm_teleop_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'so_arm_teleop' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${so_arm_teleop_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(so_arm_teleop_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${so_arm_teleop_DIR}/${_extra}")
endforeach()
