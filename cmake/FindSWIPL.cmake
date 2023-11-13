# - Try to find SWI-Prolog
# Once done this will define
#
#  SWIPL_FOUND - system has SWI-Prolog
#  SWIPL_INCLUDE_DIRS - the SWI-Prolog include directory
#  SWIPL_LIBRARIES - Link these to use SWI-Prolog
#  SWIPL_DEFINITIONS - Compiler switches required for using SWI-Prolog
#  SWIPL_ARCH - Architecture identifier used by SWI-Prolog
#
#  Copyright (c) 2019 Jan Wielemaker (jan@swi-prolog.org)
#
#  Redistribution and use is allowed according to the terms of the BSD-2
#  license.

if(NOT (SWIPL_INCLUDE_DIR AND SWIPL_LIBRARY))
  find_package(PkgConfig)
  pkg_check_modules(PC_SWIPL QUIET swipl=9)

  find_program(
      SWIPL_PROGRAM
      NAMES swipl swi-prolog)

  execute_process(
      COMMAND ${SWIPL_PROGRAM} --dump-runtime-variables
      OUTPUT_VARIABLE swipl_output)

  string(REGEX MATCHALL [^\n]+\n lines ${swipl_output})
  foreach(line IN LISTS lines)
    string(REGEX REPLACE "^PL([A-Z]+)=.*" \\1 name ${line})
    string(REGEX REPLACE "^PL[A-Z]+=\"(.*)\".*" \\1 value ${line})
    set(SWIPL_RUNTIME_${name} ${value})
    # message("SWIPL_${name} <- ${value}")
  endforeach()

  if(SWIPL_RUNTIME_ARCH)
    set(SWIPL_RUNTIME_ARCH ${SWIPL_RUNTIME_ARCH} CACHE STRING
	"Architecture subdirectory for finding foreign libraries")
  endif()

  find_path(SWIPL_INCLUDE_DIR
    NAMES
      SWI-Prolog.h
    PATHS
      ${SWIPL_RUNTIME_BASE}/include
      /usr/include
      ${CMAKE_INCLUDE_PATH}
      ${CMAKE_INSTALL_PREFIX}/include
  )

  # SWIPL_BINARY_DIR deals with SWI-Prolog running from the build directory
  get_filename_component(SWIPL_BINARY_DIR ${SWIPL_RUNTIME_BASE} DIRECTORY)
  find_library(SWIPL_LIBRARY
    NAMES
      swipl
      swipl
    PATHS
      ${SWIPL_RUNTIME_BASE}/lib/${SWIPL_RUNTIME_ARCH}
      ${SWIPL_BINARY_DIR}/src
      /usr/lib
      ${CMAKE_LIBRARY_PATH}
      ${CMAKE_INSTALL_PREFIX}/lib
  )
  mark_as_advanced(SWIPL_INCLUDE_DIR SWIPL_LIBRARY SWIPL_ARCH)

endif(NOT (SWIPL_INCLUDE_DIR AND SWIPL_LIBRARY))

# Finish up and create the final target

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SWIPL
    FOUND_VAR SWIPL_FOUND
    REQUIRED_VARS SWIPL_LIBRARY SWIPL_INCLUDE_DIR)

if(SWIPL_FOUND)
  list(APPEND SWIPL_LIBRARIES ${SWIPL_LIBRARY})
  # list(APPEND SWIPL_LIBRARIES ${SWIPL_LIBRARY}.9)
  set(SWIPL_INCLUDE_DIRS ${SWIPL_INCLUDE_DIR})
  set(SWIPL_DEFINITIONS ${PC_SWIPL_CFLAGS_OTHER})
endif()

if(SWIPL_FOUND AND NOT TARGET SWIPL::SWIPL)
  add_library(SWIPL::SWIPL UNKNOWN IMPORTED)
  set_target_properties(
      SWIPL::SWIPL PROPERTIES
      IMPORTED_LOCATION "${SWIPL_LIBRARY}"
      INTERFACE_COMPILE_OPTIONS "${PC_SWIPL_CFLAGS_OTHER}"
      INTERFACE_INCLUDE_DIRECTORIES "${SWIPL_INCLUDE_DIR}")
endif()
