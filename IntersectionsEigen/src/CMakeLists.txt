add_executable(Intersections main.cpp primitives.cpp solid.cpp contour.cpp)

target_link_libraries(Intersections )    # Link your executable with a library

target_include_directories(Intersections PUBLIC ${IntersectionsEigen_SOURCE_DIR}/include)

