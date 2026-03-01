add_executable(plan_panda plan_panda.cpp)
target_link_libraries(plan_panda PRIVATE sbf)
target_include_directories(plan_panda PRIVATE ${CMAKE_SOURCE_DIR}/include)

add_executable(benchmark_forest benchmark_forest.cpp)
target_link_libraries(benchmark_forest PRIVATE sbf)
target_include_directories(benchmark_forest PRIVATE ${CMAKE_SOURCE_DIR}/include)
