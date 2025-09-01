# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_moving_service_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED moving_service_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(moving_service_FOUND FALSE)
  elseif(NOT moving_service_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(moving_service_FOUND FALSE)
  endif()
  return()
endif()
set(_moving_service_CONFIG_INCLUDED TRUE)

# output package information
if(NOT moving_service_FIND_QUIETLY)
  message(STATUS "Found moving_service: 0.0.0 (${moving_service_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'moving_service' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${moving_service_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(moving_service_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${moving_service_DIR}/${_extra}")
endforeach()
