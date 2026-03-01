enable_testing()

# Google Test targets
set(TEST_SOURCES
    test_interval_math.cpp
    test_interval_fk.cpp
    test_node_store.cpp
    test_collision.cpp
    test_safe_box_forest.cpp
    test_graph_search.cpp
    test_planner_e2e.cpp
    test_hcache.cpp
)

foreach(TEST_SRC ${TEST_SOURCES})
    get_filename_component(TEST_NAME ${TEST_SRC} NAME_WE)
    add_executable(${TEST_NAME} ${TEST_SRC})
    target_link_libraries(${TEST_NAME} PRIVATE sbf GTest::gtest_main)
    target_include_directories(${TEST_NAME} PRIVATE
        ${CMAKE_SOURCE_DIR}/include
    )
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endforeach()
