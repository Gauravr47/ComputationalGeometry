cmake_minimum_required(VERSION 3.26)  # Specify the minimum required CMake version
project(IntersectionsEigen VERSION 1.0)                    # Name of your project

set(CMAKE_CXX_STANDARD 20)            # Use C++20 standard
set(CMAKE_CXX_STANDARD_REQUIRED True) # Enforce C++20 standard

include_directories(${IntersectionsEigen_SOURCE_DIR}/include) # Add include directory

add_subdirectory(src)
add_subdirectory(external)
add_subdirectory(testFiles)