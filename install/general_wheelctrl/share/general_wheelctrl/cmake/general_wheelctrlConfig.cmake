# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_general_wheelctrl_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED general_wheelctrl_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(general_wheelctrl_FOUND FALSE)
  elseif(NOT general_wheelctrl_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(general_wheelctrl_FOUND FALSE)
  endif()
  return()
endif()
set(_general_wheelctrl_CONFIG_INCLUDED TRUE)

# output package information
if(NOT general_wheelctrl_FIND_QUIETLY)
  message(STATUS "Found general_wheelctrl: 0.0.0 (${general_wheelctrl_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'general_wheelctrl' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${general_wheelctrl_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(general_wheelctrl_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${general_wheelctrl_DIR}/${_extra}")
endforeach()
