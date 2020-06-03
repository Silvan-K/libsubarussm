cmake_minimum_required(VERSION 3.10)

# Set the project name
project(libsubarussm)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Build output paths
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")

# Build shared libs
option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

# Add src subdirectory
add_subdirectory(src)
