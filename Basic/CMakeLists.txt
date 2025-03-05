cmake_minimum_required(VERSION 3.26)  # Specify the minimum required CMake version
project(Basics VERSION 1.0)                    # Name of your project


# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()
include_directories(${Basics_SOURCE_DIR}/include) # Add include directory

add_subdirectory(src)



