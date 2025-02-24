cmake_minimum_required(VERSION 3.24)

message(STATUS application_base_modules)
project(application_base_modules
        VERSION 1.0.0
        DESCRIPTION "Camera handling and processing"
        LANGUAGES CXX)

### Options
option(BUILD_SHARED_LIBS "Build using shared libraries" OFF)

### Configuration
if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON) # If building as shared lib on Windows
endif()

### COMPILER FLAGS ###
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_COMPILE_WARNING_AS_ERROR TRUE)

set(ENV{VCPKG_ROOT} "C:/vcpkg")

message("vcpkg root: $ENV{VCPKG_ROOT}")
### Integrate vcpkg ###
if(DEFINED ENV{VCPKG_ROOT} AND NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  set(CMAKE_TOOLCHAIN_FILE "C:/vcpkg/scripts/buildsystems/vcpkg.cmake"
      CACHE STRING "")
endif()

### create the target
set(TARGET_NAME ${PROJECT_NAME})
add_library(${TARGET_NAME})
add_library(${TARGET_NAME}::${TARGET_NAME} ALIAS ${TARGET_NAME})

### Dependencies
set(CAMERA_MODULE_PRIVATE_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set(CAMERA_MODULE_LINK_LIBRARIES)

find_package(Threads)
list(APPEND CAMERA_MODULE_LINK_LIBRARIES ${CMAKE_THREAD_LIBS_INIT})

find_package(OpenMP REQUIRED)


if(WIN32)
  list(APPEND CAMERA_MODULE_LINK_LIBRARIES wsock32 ws2_32)
endif()

# Add subdirectories for base/cpp and shared modules if not already added
if (NOT TARGET sick_visionary_cpp_base)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../base/cpp ${CMAKE_CURRENT_BINARY_DIR}/base_cpp)
endif()

if (NOT TARGET shared_cpp_utils)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../shared/cpp ${CMAKE_CURRENT_BINARY_DIR}/shared)
endif()

### Include Eigen headers directly ###
#include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../eigen)

### Set CMAKE_PREFIX_PATH to include the directory where OpenCV is located ###
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "C:/Vasu/ImageProcessing/opencv/build")

# Enable encryption

### Find OpenCV, PCL, FLANN, and Boost ###
find_package(OpenCV REQUIRED)
find_package(PCL CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(open62541 CONFIG REQUIRED)
find_package(Boost REQUIRED COMPONENTS process)

#find_package(uasdk)
# Add the following lines to find VTK and include its directories
find_package(VTK REQUIRED)

# Add VTK module autoinit
vtk_module_autoinit(
  TARGETS application_base_modules
  MODULES ${VTK_LIBRARIES}
)

### Include directories ###
include_directories(${VTK_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
if (EIGEN3_FOUND)
  include_directories(${EIGEN3_INCLUDE_DIR})
 endif()



### Link libraries ###
list(APPEND CAMERA_MODULE_LINK_LIBRARIES ${OpenCV_LIBS})
list(APPEND CAMERA_MODULE_LINK_LIBRARIES ${VTK_LIBRARIES})
list(APPEND CAMERA_MODULE_LINK_LIBRARIES ${PCL_LIBRARIES})
list(APPEND CAMERA_MODULE_LINK_LIBRARIES ${Eigen_LIBS})

#link_directories(${PCL_LIBRARY_DIRS} ${VTK_LIBRARY_DIRS})
### BUILD ###
if (BUILD_SHARED_LIBS)
  message(STATUS "A shared library will be generated")
endif()

set_target_properties(${TARGET_NAME} PROPERTIES
  CXX_STANDARD 11
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS OFF)

set (CAMERA_MODULE_SRCS
  src/Camera.cpp
  src/features.cpp
  src/Visualizer.cpp
 src/PLCData.cpp
)

set(CAMERA_MODULE_PUBLIC_HEADERS
  include/application_base_modules/Camera.h
  include/application_base_modules/features.h
  include/application_base_modules/Visualizer.h
  include/application_base_modules/PLCData.h
)

target_sources(${TARGET_NAME}
  PRIVATE
    ${CAMERA_MODULE_SRCS}
  PUBLIC FILE_SET publicHeaders
    TYPE HEADERS
    BASE_DIRS include
    FILES ${CAMERA_MODULE_PUBLIC_HEADERS}
)

target_include_directories(${TARGET_NAME}
  PUBLIC
    "$<INSTALL_INTERFACE:include>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/application_base_modules>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../base/cpp/include>"
  PRIVATE
    ${CAMERA_MODULE_PRIVATE_INCLUDE_DIRS}
)

# Link with base/cpp and shared libraries
target_link_libraries(
  ${TARGET_NAME}
  
    ${VTK_LIBRARIES}
    open62541::open62541
    sick_visionary_cpp_base::sick_visionary_cpp_base
    shared_cpp_utils
    Boost::process
    ${OpenCV_LIBS} 
    ${PCL_LIBRARIES} 
    ${Eigen_LIBS}
)
#target_link_libraries(${TARGET_NAME} ${VTK_LIBRARIES})
#target_link_libraries(${TARGET_NAME} sick_visionary_cpp_base::sick_visionary_cpp_base)
#target_link_libraries(${TARGET_NAME} shared_cpp_utils) # Replace with actual shared library target name
#target_link_libraries(${TARGET_NAME} PRIVATE open62541::open62541)
# apply options
target_compile_options(${TARGET_NAME} PRIVATE ${CAMERA_MODULE_CFLAGS})

# Installation
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
  NO_SET_AND_CHECK_MACRO
  NO_CHECK_REQUIRED_COMPONENTS_MACRO
)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  COMPATIBILITY SameMajorVersion
)

install(TARGETS ${TARGET_NAME}
  EXPORT ${PROJECT_NAME}Targets
  FILE_SET publicHeaders
)

install(
  EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
  NAMESPACE ${PROJECT_NAME}::
)

install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
