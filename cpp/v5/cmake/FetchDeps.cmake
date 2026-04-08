# ═══════════════════════════════════════════════════════════════════════════
#  FetchDeps.cmake — Download third-party dependencies via FetchContent
#
#  All sources are cached in ${CMAKE_SOURCE_DIR}/_sbf5_deps so that
#  subsequent configures do not re-download.
#
#  Libraries fetched:
#    Eigen 3.4.0          Linear algebra (header-only)
#    nlohmann/json 3.11.3 JSON serialisation (header-only)
#    doctest 2.4.11       Unit-test framework (header-only)
#    pybind11 2.12.0      C++/Python bindings
# ═══════════════════════════════════════════════════════════════════════════
include(FetchContent)
set(FETCHCONTENT_BASE_DIR "${CMAKE_SOURCE_DIR}/_sbf5_deps" CACHE PATH "" FORCE)

# -- Eigen3 --
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    eigen3
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        3.4.0
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(eigen3)

# -- nlohmann/json --
FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG        v3.11.3
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(nlohmann_json)

# -- doctest --
FetchContent_Declare(
    doctest
    GIT_REPOSITORY https://github.com/doctest/doctest.git
    GIT_TAG        v2.4.11
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(doctest)

# -- pybind11 (for Python bindings) --
FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG        v2.12.0
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(pybind11)
