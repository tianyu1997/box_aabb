# Install script for directory: C:/Users/TIAN/OneDrive - The Chinese University of Hong Kong/Desktop/code/box_aabb/safeboxforest/v5/_sbf5_deps/eigen3-src/unsupported/Eigen/CXX11

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files/SafeBoxForest_v5")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Devel" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/unsupported/Eigen/CXX11" TYPE FILE FILES
    "C:/Users/TIAN/OneDrive - The Chinese University of Hong Kong/Desktop/code/box_aabb/safeboxforest/v5/_sbf5_deps/eigen3-src/unsupported/Eigen/CXX11/Tensor"
    "C:/Users/TIAN/OneDrive - The Chinese University of Hong Kong/Desktop/code/box_aabb/safeboxforest/v5/_sbf5_deps/eigen3-src/unsupported/Eigen/CXX11/TensorSymmetry"
    "C:/Users/TIAN/OneDrive - The Chinese University of Hong Kong/Desktop/code/box_aabb/safeboxforest/v5/_sbf5_deps/eigen3-src/unsupported/Eigen/CXX11/ThreadPool"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Devel" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/unsupported/Eigen/CXX11" TYPE DIRECTORY FILES "C:/Users/TIAN/OneDrive - The Chinese University of Hong Kong/Desktop/code/box_aabb/safeboxforest/v5/_sbf5_deps/eigen3-src/unsupported/Eigen/CXX11/src" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

