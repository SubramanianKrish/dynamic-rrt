cmake_minimum_required(VERSION 3.5)

# Make sure that the project is not built in the root directory and is built inside /build folder
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
    message(FATAL_ERROR "Do not build in-source. Please remove CMakeCache.txt and the CmakeFiles/ directory. Then build out-of source")
endif()

# Set C++ Standard
set(CMAKE_CXX_STANDARD 11)

# Create any folders if not present
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Make the library directory (Can hold .so or .a files)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

# Include OpenCV and Eigen
find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV > 3.0 is not found. Please check readme")
endif()

# Add libraries - tells CMake to look for headers here when compiling
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    )

# Create a static library (compiled and linked completely -> One big executable)
add_library(${PROJECT_NAME} STATIC 
    ${CMAKE_SOURCE_DIR}/src/main.cpp
    ${CMAKE_SOURCE_DIR}/src/viewer.cpp
    ${CMAKE_SOURCE_DIR}/src/state.cpp
    ${CMAKE_SOURCE_DIR}/src/Obstacle.cpp
    ${CMAKE_SOURCE_DIR}/src/Map.cpp
    )

# Link external libraries (opencv, eigen and our custom library)
target_link_libraries(
    ${PROJECT_NAME}
    ${OpenCV_LIBS}
    )

# Add main executable file
add_executable(main ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)
target_link_libraries(main PUBLIC ${PROJECT_NAME})

# Add unit_test executables
# add_executable(map ${CMAKE_CURRENT_SOURCE_DIR}/testMap.cpp)
# target_link_libraries(map PUBLIC ${PROJECT_NAME})
