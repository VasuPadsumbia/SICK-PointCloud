#
# Copyright (c) 2024 SICK AG, Waldkirch
#
# SPDX-License-Identifier: Unlicense

cmake_minimum_required(VERSION 3.24)

project(spng_miniz
        VERSION 1.0.0
        DESCRIPTION "static spng compiled against miniz"
        LANGUAGES C)

### Configuration
if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON) # If building as shared lib on Windows
endif()

add_library(spng_miniz spng/spng.c miniz/miniz.c)
target_include_directories(spng_miniz 
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/spng>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/miniz>
        $<INSTALL_INTERFACE:include/spng>
        $<INSTALL_INTERFACE:include/miniz>
)
target_compile_definitions(spng_miniz PRIVATE SPNG_USE_MINIZ PUBLIC SPNG_STATIC=ON)


# Export the target
install(TARGETS spng_miniz
  EXPORT spng_minizTargets
  FILE_SET publicHeaders
)

install(
  EXPORT spng_minizTargets
  FILE spng_minizTargets.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/spng_miniz
  NAMESPACE spng_miniz::
)

install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/spng_minizConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/spng_minizConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/spng_miniz
)