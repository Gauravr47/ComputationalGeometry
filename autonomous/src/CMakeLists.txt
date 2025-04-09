add_executable(autonomous main.cpp adts.cpp)

target_include_directories(autonomous PUBLIC ${autonomous_SOURCE_DIR}/include)

target_link_libraries(autonomous cg)