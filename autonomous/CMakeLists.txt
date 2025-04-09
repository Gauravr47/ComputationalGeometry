cmake_minimum_required(VERSION 3.26)  # Specify the minimum required CMake version
project(autonomous VERSION 1.0)                    # Name of your project

add_library(cg STATIC IMPORTED [GLOBAL])

set_target_properties(cg PROPERTIES
    IMPORTED_LOCATION "${autonomous_SOURCE_DIR}/../Basic/build2/lib/Release/cg.lib"
    INTERFACE_INCLUDE_DIRECTORIES "${autonomous_SOURCE_DIR}/../Basic/include"
)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

# Set C++ version
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Output directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_subdirectory(src)
add_subdirectory(testFiles)


