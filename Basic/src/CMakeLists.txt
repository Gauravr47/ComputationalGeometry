add_library(cg STATIC utilFuncs.cpp basic_data_structures.cpp geometry.cpp algorithms.cpp gadts.cpp solid.cpp)

add_executable(Basics main.cpp)

target_include_directories(Basics PUBLIC ${BASIC_SOURCE_DIR}/include)

target_link_libraries(Basics PRIVATE cg)    # Link your executable with a library