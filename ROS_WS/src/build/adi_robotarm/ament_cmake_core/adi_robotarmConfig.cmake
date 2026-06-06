# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_adi_robotarm_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED adi_robotarm_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(adi_robotarm_FOUND FALSE)
  elseif(NOT adi_robotarm_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(adi_robotarm_FOUND FALSE)
  endif()
  return()
endif()
set(_adi_robotarm_CONFIG_INCLUDED TRUE)

# output package information
if(NOT adi_robotarm_FIND_QUIETLY)
  message(STATUS "Found adi_robotarm: 0.0.1 (${adi_robotarm_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'adi_robotarm' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT adi_robotarm_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(adi_robotarm_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${adi_robotarm_DIR}/${_extra}")
endforeach()
