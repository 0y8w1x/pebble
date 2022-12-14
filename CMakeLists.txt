cmake_minimum_required(VERSION 3.10...3.18)

project(basalt)

set(TARGET_OS "linux")

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/" ${CMAKE_MODULE_PATH})

if (${TARGET_OS} STREQUAL "linux")
  if(NOT EIGEN_ROOT)
    set(EIGEN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/basalt-headers/thirdparty/eigen")
  endif()
endif()

if (${TARGET_OS} STREQUAL "linux")
  string(TOLOWER "${PROJECT_NAME}" PROJECT_NAME_LOWERCASE)
  find_program(DPKG_PROGRAM dpkg DOC "dpkg program of Debian-based systems")
  if(DPKG_PROGRAM)
    execute_process(
      COMMAND ${DPKG_PROGRAM} --print-architecture
      OUTPUT_VARIABLE CPACK_DEBIAN_PACKAGE_ARCHITECTURE
      OUTPUT_STRIP_TRAILING_WHITESPACE)
  endif(DPKG_PROGRAM)


  find_program(LSB_RELEASE_PROGRAM lsb_release DOC "lsb_release program of Debian-based systems")
  if(LSB_RELEASE_PROGRAM)
    execute_process(COMMAND ${LSB_RELEASE_PROGRAM} -rs
      OUTPUT_VARIABLE LSB_RELEASE_ID_SHORT
      OUTPUT_STRIP_TRAILING_WHITESPACE)

    if(${LSB_RELEASE_ID_SHORT} EQUAL "20.04")
      set(DEBIAN_DEPENDS "libtbb2, liblz4-1, libbz2-1.0, libboost-filesystem1.71.0, libboost-date-time1.71.0, libboost-program-options1.71.0, libboost-regex1.71.0, libopencv-dev, libglew2.1, libjpeg8, libpng16-16, librealsense2, librealsense2-dkms, librealsense2-gl, librealsense2-utils")

    elseif(${LSB_RELEASE_ID_SHORT} EQUAL "18.04")
      set(DEBIAN_DEPENDS "libtbb2, liblz4-1, libbz2-1.0, libboost-filesystem1.65.1, libboost-date-time1.65.1, libboost-program-options1.65.1, libboost-regex1.65.1, libopencv-dev, libglew2.0, libjpeg8, libpng16-16, librealsense2, librealsense2-dkms, librealsense2-gl, librealsense2-utils")

    elseif(${LSB_RELEASE_ID_SHORT} EQUAL "16.04")
      set(DEBIAN_DEPENDS "libtbb2, liblz4-1, libbz2-1.0, libboost-filesystem1.58.0, libboost-date-time1.58.0, libboost-program-options1.58.0, libboost-regex1.58.0, libopencv-dev, libglew1.13, libjpeg8, libpng12-0, libstdc++6, librealsense2, librealsense2-dkms, librealsense2-gl, librealsense2-utils")
    endif()

  endif(LSB_RELEASE_PROGRAM)

  string(TIMESTAMP PROJECT_VERSION_REVISION "%Y%m%d%H%M")

  set(CPACK_GENERATOR "DEB")
  set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Vladyslav Usenko <vlad.usenko@tum.de>")
  set(CPACK_PACKAGE_VERSION_MAJOR "0")
  set(CPACK_PACKAGE_VERSION_MINOR "1")
  set(CPACK_PACKAGE_VERSION_PATCH "0-${PROJECT_VERSION_REVISION}~${LSB_RELEASE_ID_SHORT}")
  set(CPACK_DEBIAN_PACKAGE_DEPENDS ${DEBIAN_DEPENDS})
  set(CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME_LOWERCASE}_${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}_${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
  set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
  include(CPack)

  # Configure CCache if available
  if (NOT CMAKE_C_COMPILER_LAUNCHER AND NOT CMAKE_CXX_COMPILER_LAUNCHER)
    find_program(CCACHE_PROGRAM ccache)
    if(CCACHE_PROGRAM)
      message(STATUS "Found ccache: ${CCACHE_PROGRAM}")
      set(CMAKE_C_COMPILER_LAUNCHER   ${CCACHE_PROGRAM})
      set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
    else()
      message(STATUS "Dind't find ccache")
    endif()
  else()
    message(STATUS "Compiler launcher already set. Not configuring ccache.")
    message(STATUS "CMAKE_C_COMPILER_LAUNCHER: ${CMAKE_C_COMPILER_LAUNCHER}")
    message(STATUS "CMAKE_CXX_COMPILER_LAUNCHER: ${CMAKE_CXX_COMPILER_LAUNCHER}")
  endif()
endif()

if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if (${TARGET_OS} STREQUAL "linux")
  # Flags used for CHECK_CXX_SOURCE_COMPILES
  set(CMAKE_REQUIRED_FLAGS "-Wno-error")

  # save flags passed by user
  set(BASALT_PASSED_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

  set(CMAKE_CXX_FLAGS_DEBUG  "-O0 -g -DEIGEN_INITIALIZE_MATRICES_BY_NAN")          # cmake default: "-g"
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DEIGEN_INITIALIZE_MATRICES_BY_NAN")  # cmake default: "-O2 -g -DNDEBUG"
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")                                      # cmake default: "-O3 -DNDEBUG"
  set(CMAKE_CXX_FLAGS_CIDEBUG  "-O0 -DEIGEN_INITIALIZE_MATRICES_BY_NAN")          # CI version with no debug symbols
  set(CMAKE_CXX_FLAGS_CIRELWITHDEBINFO "-O3 -DEIGEN_INITIALIZE_MATRICES_BY_NAN")  # CI version with no debug symbols

  # base set of compile flags
  set(BASALT_CXX_FLAGS "-Wall -Wextra -Werror -Wno-error=unused-parameter -ftemplate-backtrace-limit=0")
endif()

# clang-specific compile flags
if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
  set(BASALT_CXX_FLAGS "${BASALT_CXX_FLAGS} -Wno-exceptions -fcolor-diagnostics -frelaxed-template-template-args -Wno-error=deprecated-declarations")

  if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 10)
      # These are disabled to avoid lot's of warnings in Eigen code with clang 10
      set(BASALT_CXX_FLAGS "${BASALT_CXX_FLAGS} -Wno-misleading-indentation -Wno-error=deprecated-copy")
  endif()

  #   - Added TBB_USE_GLIBCXX_VERSION macro to specify the version of GNU
  #     libstdc++ when it cannot be properly recognized, e.g. when used
  #     with Clang on Linux* OS. Adopted from https://github.com/wjakob/tbb
  if(NOT TBB_USE_GLIBCXX_VERSION AND UNIX AND NOT APPLE)
    if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
      string(REPLACE "." "0" TBB_USE_GLIBCXX_VERSION ${CMAKE_CXX_COMPILER_VERSION})
    endif()
    add_definitions(-DTBB_USE_GLIBCXX_VERSION=${TBB_USE_GLIBCXX_VERSION})
  endif()
else()
  set(BASALT_CXX_FLAGS "${BASALT_CXX_FLAGS} -Wno-error=maybe-uninitialized -Wno-error=implicit-fallthrough")

  if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 9)
      # These are disabled to avoid lot's of warnings in Eigen code with gcc-9
      set(BASALT_CXX_FLAGS "${BASALT_CXX_FLAGS} -Wno-error=deprecated-copy")
  endif()
  if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 10.0)
    # These are disabled due to warnings in pangolin
    #set(BASALT_CXX_FLAGS "${BASALT_CXX_FLAGS} -Wno-error=parentheses")
    set(BASALT_CXX_FLAGS "${BASALT_CXX_FLAGS} -Wno-parentheses")
  endif()
endif()



# OpenMP option and compile flags
#
# Note: OpenMP and TBB don't mix well, so we disable Eigen's parallelization.
# It's trying to paralellize matrix products during SC, which we run in a parallel_reduce using TBB.
# Turns out using OpenMP can slow down the computby factor 10-100x! So for now we discable it completely.
# One place where Eigen's parallelization could still have been useful is the CG solver in the mapper.
# We could in the future investiagte other implementations (paralellized with TBB) or selectively enabling
# Eigen's parallelization just for CG, setting number of threads to 1 everywhere else.
# Another way to ensure Eigen doesn't use OpenMP regardless of how it was built is setting the environment
# variable OMP_NUM_THREADS=1 beofre running the application.
#
# See: https://eigen.tuxfamily.org/dox/TopicMultiThreading.html
#
# If we enable BLAS / LAPACK either directly or via thirdparty libs like ceres,
# make sure to disable OpenMP for the linked BLAS library. In particular on Ubuntu it seems OpenBLAS is often installed,
# and it can have similar issues in multithreaded applications if it's own parallelization with OpenMP is enabled.
# You can set the environment varaibles OPENBLAS_NUM_THREADS=1 or OMP_NUM_THREADS=1. This is also mentioned in the ceres
# installation documentation.
#
# See also: https://github.com/xianyi/OpenBLAS/wiki/faq#multi-threaded
#
# Set EIGEN_DONT_PARALLELIZE to be sure it doesn't use openmp,
# just in case some dependency enables openmp without us noticing.
if (${TARGET_OS} STREQUAL "linux")
  set(BASALT_CXX_FLAGS "${BASALT_CXX_FLAGS} -DEIGEN_DONT_PARALLELIZE")
endif()

#option(USE_OPENMP "Use OpenMP (e.g. for parallel computation in Eigen)" ${USE_OPENMP_DEFAULT})
#if(USE_OPENMP)
#  message(STATUS "OpenMP Enabled")
#  set(BASALT_CXX_FLAGS "${BASALT_CXX_FLAGS} -fopenmp")
#else()
#  message(STATUS "OpenMP Disabled")
#endif()

# Make specialization for double / float optional.
# Disabling them can be useful for better compile times during development.
option(BASALT_INSTANTIATIONS_DOUBLE "Instatiate templates for Scalar=double." ON)
option(BASALT_INSTANTIATIONS_FLOAT "Instatiate templates for Scalar=float." ON)

if(BASALT_INSTANTIATIONS_DOUBLE)
  list(APPEND BASALT_COMPILE_DEFINITIONS BASALT_INSTANTIATIONS_DOUBLE)
endif()
if(BASALT_INSTANTIATIONS_FLOAT)
  list(APPEND BASALT_COMPILE_DEFINITIONS BASALT_INSTANTIATIONS_FLOAT)
endif()

# setup combined compiler flags
set(CMAKE_CXX_FLAGS "${BASALT_CXX_FLAGS} ${BASALT_MARCH_FLAGS} ${BASALT_PASSED_CXX_FLAGS}")

if (${TARGET_OS} STREQUAL "android")
  set(EIGEN3_INCLUDE_DIR ${EIGEN_ROOT})
  set(EIGEN_INCLUDE_DIR_HINTS ${EIGEN_ROOT})
  find_package(Eigen3 3.4.0 EXACT REQUIRED MODULE)
  message(STATUS "Found Eigen headers in: ${EIGEN3_INCLUDE_DIR}")
  if(NOT EIGEN3_INCLUDE_DIR MATCHES "^${EIGEN_ROOT}")
    message(WARNING "Found Eigen headers are outside of specified EIGEN_ROOT '${EIGEN_ROOT}'")
  endif()
endif()

if (${TARGET_OS} STREQUAL "linux")
  find_package(TBB REQUIRED)
  message(STATUS "Found TBB ${TBB_VERSION_MAJOR}.${TBB_VERSION_MINOR} (interface version ${TBB_INTERFACE_VERSION}) headers in: ${TBB_INCLUDE_DIRS}")
  if (TBB_INTERFACE_VERSION LESS 11004)
  # enable global_control header for earlier TBB versions (Ubuntu 16.04, 18.04)
  add_definitions(-DTBB_PREVIEW_GLOBAL_CONTROL)
  endif()
endif()

if (${TARGET_OS} STREQUAL "android")
  set(OpenCV_DIR "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/opencv-android-sdk/sdk/native/jni")
endif()

# NOTE: not specifying version, since 2, 3 or 4 is fine
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d highgui)
message(STATUS "Found OpenCV ${OpenCV_VERSION} headers in: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "Found OpenCV_LIBS: ${OpenCV_LIBS}")

# Add our own custom scoped opencv target since none is provided by OpenCV itself
add_library(basalt::opencv INTERFACE IMPORTED)
set_property(TARGET basalt::opencv PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${OpenCV_INCLUDE_DIRS})
set_property(TARGET basalt::opencv PROPERTY INTERFACE_LINK_LIBRARIES ${OpenCV_LIBS})

# skip boost dependency
set(SKIP_PERFORMANCE_COMPARISON ON)
if (${TARGET_OS} STREQUAL "android")
  set(BASALT_BUILTIN_EIGEN OFF)
endif()
set(JUST_INSTALL_CEREAL ON)
set(EIGEN_BUILD_DOC OFF)

if (${TARGET_OS} STREQUAL "android")
  add_subdirectory(thirdparty/eigen)
endif()
add_subdirectory(thirdparty)

# main library
add_library(basalt SHARED)

# List all header and source files with target_sources.
# This ensures, e.g., that QtCreator >= 6.0 properly recognize these headers to belong to the project.
# To support cmake < 3.13, use absolute paths (see: https://crascit.com/2016/01/31/enhanced-source-file-handling-with-target_sources/)
target_sources(basalt
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/device/rs_t265.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/hash_bow/hash_bow.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/io/dataset_io.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/linearization/block_diagonal.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/linearization/imu_block.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/linearization/landmark_block.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/linearization/landmark_block_abs_dynamic.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/linearization/linearization_abs_qr.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/linearization/linearization_abs_sc.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/linearization/linearization_base.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/linearization/linearization_rel_sc.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/optical_flow/frame_to_frame_optical_flow.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/optical_flow/multiscale_frame_to_frame_optical_flow.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/optical_flow/optical_flow.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/optical_flow/patch.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/optical_flow/patch_optical_flow.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/optical_flow/patterns.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/optimization/accumulator.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/optimization/linearize.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/optimization/poses_linearize.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/optimization/poses_optimize.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/optimization/spline_linearize.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/optimization/spline_optimize.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/utils/ba_utils.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/utils/cast_utils.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/utils/common_types.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/utils/format.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/utils/imu_types.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/utils/keypoints.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/utils/nfr.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/utils/sim_utils.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/utils/system_utils.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/utils/test_utils.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/utils/time_utils.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/utils/tracks.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/utils/union_find.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/utils/vio_config.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/vi_estimator/ba_base.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/vi_estimator/landmark_database.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/vi_estimator/marg_helper.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/vi_estimator/nfr_mapper.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/vi_estimator/sc_ba_base.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/vi_estimator/sqrt_ba_base.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/vi_estimator/sqrt_keypoint_vio.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/vi_estimator/sqrt_keypoint_vo.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/basalt/vi_estimator/vio_estimator.h
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/io/marg_data_io.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/linearization/landmark_block.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/linearization/linearization_abs_qr.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/linearization/linearization_abs_sc.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/linearization/linearization_base.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/linearization/linearization_rel_sc.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/optical_flow/optical_flow.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils/keypoints.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils/system_utils.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils/time_utils.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils/vio_config.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/vi_estimator/ba_base.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/vi_estimator/landmark_database.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/vi_estimator/marg_helper.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/vi_estimator/nfr_mapper.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/vi_estimator/sc_ba_base.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/vi_estimator/sqrt_ba_base.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/vi_estimator/sqrt_keypoint_vio.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/vi_estimator/sqrt_keypoint_vo.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/vi_estimator/vio_estimator.cpp
)

if (${TARGET_OS} STREQUAL "linux")
  set(external_libs TBB::tbb basalt::basalt-headers)
elseif(${TARGET_OS} STREQUAL "android")
  set(external_libs "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/tbb/libs/android/${ANDROID_ABI}/libtbb.so Eigen3::Eigen")
  set(tbb_include "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/tbb/include")
endif()

target_link_libraries(basalt PUBLIC basalt::opencv ${external_libs})
target_include_directories(basalt PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${tbb_include})
target_compile_definitions(basalt PUBLIC ${BASALT_COMPILE_DEFINITIONS})

find_package(realsense2 QUIET)
if(realsense2_FOUND)
  add_executable(basalt_rs_t265_vio src/rs_t265_vio.cpp src/device/rs_t265.cpp)
  target_link_libraries(basalt_rs_t265_vio basalt realsense2::realsense2)
endif()
