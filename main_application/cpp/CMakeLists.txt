#
# Copyright (c) 2024 SICK AG, Waldkirch
#
# SPDX-License-Identifier: Unlicense

cmake_minimum_required(VERSION 3.24)

message(STATUS "project: Object_Processor")
project(Object_Processor
        VERSION 1.0.0
        DESCRIPTION "Object points computation of the visionary programming API"
        LANGUAGES CXX C)

### COMPILER FLAGS ###
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

### Configuration
if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON) # If building as shared lib on Windows
endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

### Dependencies
#find_package(open62541 REQUIRED)
#find_package(VTK REQUIRED)
#find_package(PCL REQUIRED)
#find_package(OpenCV REQUIRED)

# root folder and name of the application_base_modules module
set(APPLICATION_BASE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../Application/base)
set(APPLICATION_BASE_NAME application_base_modules::application_base_modules)

# device base interface, if not already added
if (NOT TARGET ${APPLICATION_BASE_NAME})
  add_subdirectory(${APPLICATION_BASE_PATH} ${CMAKE_CURRENT_BINARY_DIR}/Application)
endif()

add_executable(ObjectProcessor
ObjectProcessor.cpp)
add_executable(OPCTest
OPCTest.cpp)
add_executable(comm_worker Communication.cpp)
add_executable(pos_worker PositionData.cpp)
add_executable(main_app app.cpp )

#add_custom_command(TARGET main_app pos_worker comm_worker ObjectProcessor OPCTest POST_BUILD
#  COMMAND ${CMAKE_COMMAND} -E rm -f ${CMAKE_BINARY_DIR}/main_application/cpp/*.dll
#)
#file(GLOB DLL_FILES "${CMAKE_BINARY_DIR}/main_application/cpp/*.dll")
#
#foreach(DLL ${DLL_FILES})
#    add_custom_command(TARGET main_app pos_worker comm_worker ObjectProcessor OPCTest POST_BUILD
#        COMMAND ${CMAKE_COMMAND} -E copy_if_different
#        ${DLL} ${CMAKE_BINARY_DIR}/Application/base/cpp
#        COMMENT "Moving ${DLL} to Application/base/cpp"
#    )
#endforeach()
#
#foreach(DLL ${DLL_FILES})
#    add_custom_command(TARGET main_app pos_worker comm_worker ObjectProcessor OPCTest POST_BUILD
#        COMMAND ${CMAKE_COMMAND} -E remove ${DLL}
#        COMMENT "Deleting ${DLL} from main_application/cpp"
#    )
#endforeach()

target_include_directories(ObjectProcessor
  PRIVATE )#${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

target_include_directories(OPCTest
  PRIVATE )#${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

target_include_directories(comm_worker
  PRIVATE )#${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

target_include_directories(pos_worker
  PRIVATE )#${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

target_include_directories(main_app
  PRIVATE)# ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

target_link_libraries(ObjectProcessor
  ${APPLICATION_BASE_NAME}
  spng_miniz
  #open62541::open62541
  #${VTK_LIBRARIES}
  #${PCL_LIBRARIES}
  #${OpenCV_LIBRARIES}
)
target_link_libraries(OPCTest
  ${APPLICATION_BASE_NAME}
  spng_miniz
  #open62541::open62541
  #${VTK_LIBRARIES}
  #${PCL_LIBRARIES}
  #${OpenCV_LIBRARIES}

)

target_link_libraries(comm_worker
    ${APPLICATION_BASE_NAME}
    spng_miniz
)

target_link_libraries(pos_worker 
    ${APPLICATION_BASE_NAME}
    spng_miniz
)
target_link_libraries(main_app
  ${APPLICATION_BASE_NAME}
  spng_miniz
  #open62541::open62541
  #${VTK_LIBRARIES}
  #${PCL_LIBRARIES}
  #${OpenCV_LIBRARIES}
)

#set_target_properties(ObjectProcessor OPCTest comm_worker pos_worker main_app PROPERTIES
#  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Application/base/cpp"
#  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Application/base/cpp"
#  ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Application/base/cpp"
#)