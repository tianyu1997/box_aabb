cmake_minimum_required(VERSION 3.16)
project(SafeBoxForest VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ─── Options ─────────────────────────────────────────────────────────────────
option(SBF_WITH_DRAKE  "Enable Drake GCS adapter"       OFF)
option(SBF_WITH_OMPL   "Enable OMPL adapter"            OFF)
option(SBF_WITH_PYTHON "Build pybind11 Python bindings"  OFF)
option(SBF_BUILD_TESTS "Build unit tests"                ON)
option(SBF_BUILD_EXAMPLES "Build example programs"       ON)
option(SBF_BUILD_EXPERIMENTS "Build experiment programs"   ON)

# ─── Compiler flags ──────────────────────────────────────────────────────────
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -O3 -march=native)
endif()

# ─── CMake module path ───────────────────────────────────────────────────────
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# ─── Required dependencies ───────────────────────────────────────────────────
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
message(STATUS "Eigen3 found: ${EIGEN3_INCLUDE_DIR}")

# nlohmann/json (header-only)
include(FetchContent)
FetchContent_Declare(json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG        v3.11.3
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(json)

# ─── Optional: Drake ─────────────────────────────────────────────────────────
if(SBF_WITH_DRAKE)
    find_package(drake REQUIRED)
    message(STATUS "Drake found: ${drake_DIR}")
    add_compile_definitions(SBF_WITH_DRAKE)
endif()

# ─── Optional: OMPL ──────────────────────────────────────────────────────────
if(SBF_WITH_OMPL)
    find_package(ompl REQUIRED)
    message(STATUS "OMPL found: ${ompl_DIR}")
    add_compile_definitions(SBF_WITH_OMPL)
endif()

# ─── Core library ────────────────────────────────────────────────────────────
set(SBF_SOURCES
    src/core/robot.cpp
    src/aabb/interval_math.cpp
    src/aabb/interval_fk.cpp
    src/aabb/fk_scalar.cpp
    src/forest/collision.cpp
    src/forest/node_store.cpp
    src/forest/hier_aabb_tree.cpp
    src/forest/safe_box_forest.cpp
    src/forest/deoverlap.cpp
    src/forest/connectivity.cpp
    src/forest/coarsen.cpp
    src/planner/graph_search.cpp
    src/planner/connector.cpp
    src/planner/path_smoother.cpp
    src/planner/sbf_planner.cpp
    src/planner/pipeline.cpp
    src/io/hcache.cpp
    src/io/json_io.cpp
    src/adapters/drake_gcs.cpp
    src/adapters/ompl_adapter.cpp
)

add_library(sbf STATIC ${SBF_SOURCES})
target_include_directories(sbf PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(sbf PUBLIC Eigen3::Eigen nlohmann_json::nlohmann_json)

if(SBF_WITH_DRAKE)
    target_link_libraries(sbf PUBLIC drake::drake)
endif()
if(SBF_WITH_OMPL)
    target_link_libraries(sbf PUBLIC ompl::ompl)
endif()

# ─── Python bindings ─────────────────────────────────────────────────────────
if(SBF_WITH_PYTHON)
    add_subdirectory(python)
endif()

# ─── Tests ───────────────────────────────────────────────────────────────────
if(SBF_BUILD_TESTS)
    enable_testing()
    FetchContent_Declare(googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG        v1.14.0
        GIT_SHALLOW    TRUE
    )
    FetchContent_MakeAvailable(googletest)
    add_subdirectory(tests)
endif()

# ─── Examples ────────────────────────────────────────────────────────────────
if(SBF_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# ─── Experiments ─────────────────────────────────────────────────────────────
if(SBF_BUILD_EXPERIMENTS)
    add_subdirectory(experiments)
endif()

# ─── Install ─────────────────────────────────────────────────────────────────
install(TARGETS sbf
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    INCLUDES DESTINATION include
)
install(DIRECTORY include/sbf DESTINATION include)
