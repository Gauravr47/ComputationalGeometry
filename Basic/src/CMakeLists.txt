add_executable(Basics main.cpp utilFuncs.cpp node.cpp geometry.cpp algorithms.cpp)

target_link_libraries(Basics)    # Link your executable with a library

target_include_directories(Basics PUBLIC ${IntersectionsEigen_SOURCE_DIR}/include)


