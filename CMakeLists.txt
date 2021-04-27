# Minimum version of CMake required to build this project
cmake_minimum_required(VERSION 3.0)

# Make sure that the project is not built in the root directory and is built inside /build folder
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
message(FATAL_ERROR "Do not build in-source. Please remove CMakeCache.txt and the CmakeFiles/ directory. Then build out-of source")
endif()

# Specify project name and description
project(dynamicRRT)

# Set C++ Standard to C++11
set(CMAKE_CXX_STANDARD 11)

# Link to pthreads library for multithreading
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

# Create any folders if not present
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Ensure that the executables are created in the bin dir
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# Add subdirectory with a sub-project
add_subdirectory(${CMAKE_SOURCE_DIR}/src)

# Specify build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Show all warnings during compilation
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
