cmake_minimum_required(VERSION 3.24)
project(mvslam VERSION 0.1.0 LANGUAGES C CXX
        DESCRIPTION "Modular multi-mode (mono/stereo/RGB-D x IMU) visual SLAM core")

# ---------------------------------------------------------------------------
# Design references: docs/design/07-build-platforms.md
#  * All third-party code comes from tag-pinned git submodules (third_party/).
#  * Our targets are C++20 with extensions off; third-party keeps its defaults.
#  * No -march=native anywhere (portable binaries for Jetson / Apple Silicon / CI).
# ---------------------------------------------------------------------------
list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# All executables (tests, apps) land in build/<preset>/bin regardless of how OpenCV is provided
# (an in-tree OpenCV build sets the same directory itself).
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

include(Options)        # MVSLAM_* options + consistency checks
include(Warnings)       # mvslam_set_warnings(<target>)
include(Sanitizers)     # MVSLAM_SANITIZER
include(Dependencies)   # submodule wiring (Eigen, Sophus, Ceres, g2o, GTSAM, yaml-cpp, spdlog, cereal, gtest)
include(OpenCV)         # MVSLAM_OPENCV_SOURCE = auto|system|submodule|none

add_subdirectory(src)
if(MVSLAM_BUILD_APPS AND NOT IOS)
  add_subdirectory(apps)
endif()
if(MVSLAM_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

message(STATUS "mvslam ${PROJECT_VERSION}: ceres=${MVSLAM_WITH_CERES} g2o=${MVSLAM_WITH_G2O} gtsam=${MVSLAM_WITH_GTSAM} "
               "opencv=${MVSLAM_OPENCV_SOURCE}(${MVSLAM_HAVE_OPENCV}) tests=${MVSLAM_BUILD_TESTS} sanitizer=${MVSLAM_SANITIZER}")
