# Wires the tag-pinned submodules in third_party/ into CMake targets.
# One Eigen for everything: our code, Ceres, g2o and GTSAM all resolve find_package(Eigen3)
# to the submodule through CMAKE_FIND_PACKAGE_REDIRECTS_DIR (CMake >= 3.24).
set(MVSLAM_THIRD_PARTY "${CMAKE_CURRENT_SOURCE_DIR}/third_party")
foreach(_sm eigen sophus yaml-cpp spdlog cereal googletest)
  if(NOT EXISTS "${MVSLAM_THIRD_PARTY}/${_sm}/CMakeLists.txt" AND NOT EXISTS "${MVSLAM_THIRD_PARTY}/${_sm}/sophus")
    message(FATAL_ERROR "Submodule third_party/${_sm} is empty. Run: git submodule update --init --recursive --depth 1")
  endif()
endforeach()

# ---- Eigen (header-only, 5.0.1) -------------------------------------------------------------
set(MVSLAM_EIGEN_DIR "${MVSLAM_THIRD_PARTY}/eigen")
file(WRITE "${CMAKE_FIND_PACKAGE_REDIRECTS_DIR}/eigen3-config.cmake" "
if(NOT TARGET Eigen3::Eigen)
  add_library(Eigen3::Eigen INTERFACE IMPORTED)
  set_target_properties(Eigen3::Eigen PROPERTIES INTERFACE_INCLUDE_DIRECTORIES \"${MVSLAM_EIGEN_DIR}\")
endif()
set(EIGEN3_INCLUDE_DIR \"${MVSLAM_EIGEN_DIR}\")
set(EIGEN3_INCLUDE_DIRS \"${MVSLAM_EIGEN_DIR}\")
set(EIGEN3_VERSION_STRING \"5.0.1\")
set(EIGEN3_FOUND TRUE)
set(Eigen3_FOUND TRUE)
")
file(WRITE "${CMAKE_FIND_PACKAGE_REDIRECTS_DIR}/eigen3-config-version.cmake" "
set(PACKAGE_VERSION \"5.0.1\")
if(PACKAGE_FIND_VERSION VERSION_GREATER PACKAGE_VERSION)
  set(PACKAGE_VERSION_COMPATIBLE FALSE)
else()
  set(PACKAGE_VERSION_COMPATIBLE TRUE)
endif()
")
find_package(Eigen3 REQUIRED)  # resolves to the redirect above

# ---- Sophus (header-only, 1.24.6) -----------------------------------------------------------
add_library(mvslam_sophus INTERFACE)
add_library(mvslam::sophus ALIAS mvslam_sophus)
target_include_directories(mvslam_sophus SYSTEM INTERFACE "${MVSLAM_THIRD_PARTY}/sophus")
target_compile_definitions(mvslam_sophus INTERFACE SOPHUS_USE_BASIC_LOGGING=1)
target_link_libraries(mvslam_sophus INTERFACE Eigen3::Eigen)

# ---- cereal (header-only, 1.3.2) ------------------------------------------------------------
add_library(mvslam_cereal INTERFACE)
add_library(mvslam::cereal ALIAS mvslam_cereal)
target_include_directories(mvslam_cereal SYSTEM INTERFACE "${MVSLAM_THIRD_PARTY}/cereal/include")
target_compile_definitions(mvslam_cereal INTERFACE CEREAL_THREAD_SAFE=1)

# ---- yaml-cpp (0.9.0) -----------------------------------------------------------------------
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "" FORCE)
set(YAML_CPP_INSTALL OFF CACHE BOOL "" FORCE)
set(YAML_CPP_FORMAT_SOURCE OFF CACHE BOOL "" FORCE)
set(YAML_BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
add_subdirectory("${MVSLAM_THIRD_PARTY}/yaml-cpp" EXCLUDE_FROM_ALL)

# ---- spdlog (1.17.0) ------------------------------------------------------------------------
set(SPDLOG_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(SPDLOG_BUILD_EXAMPLE OFF CACHE BOOL "" FORCE)
set(SPDLOG_INSTALL OFF CACHE BOOL "" FORCE)
add_subdirectory("${MVSLAM_THIRD_PARTY}/spdlog" EXCLUDE_FROM_ALL)

# ---- googletest (1.18.0) --------------------------------------------------------------------
if(MVSLAM_BUILD_TESTS)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  set(BUILD_GMOCK ON CACHE BOOL "" FORCE)
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  add_subdirectory("${MVSLAM_THIRD_PARTY}/googletest" EXCLUDE_FROM_ALL)
endif()

# ---- benchmark (1.9.5, optional) ------------------------------------------------------------
if(MVSLAM_BUILD_BENCH)
  set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
  set(BENCHMARK_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
  add_subdirectory("${MVSLAM_THIRD_PARTY}/benchmark" EXCLUDE_FROM_ALL)
endif()

# ---- Ceres (2.2.0) --------------------------------------------------------------------------
if(MVSLAM_WITH_CERES)
  # Same choices as VINS_Remastered (portable, iOS-friendly): Eigen sparse only, no SuiteSparse/LAPACK,
  # built-in miniglog instead of glog/gflags.
  set(MINIGLOG ON CACHE BOOL "" FORCE)
  set(MINIGLOG_MAX_LOG_LEVEL 0 CACHE STRING "" FORCE)
  set(EIGENSPARSE ON CACHE BOOL "" FORCE)
  set(SUITESPARSE OFF CACHE BOOL "" FORCE)
  set(CXSPARSE OFF CACHE BOOL "" FORCE)
  set(EIGENMETIS OFF CACHE BOOL "" FORCE)
  set(LAPACK OFF CACHE BOOL "" FORCE)
  set(CUSTOM_BLAS OFF CACHE BOOL "" FORCE)
  set(USE_CUDA OFF CACHE BOOL "" FORCE)
  set(GFLAGS OFF CACHE BOOL "" FORCE)
  set(SCHUR_SPECIALIZATIONS ${MVSLAM_CERES_SCHUR_SPECIALIZATIONS} CACHE BOOL "" FORCE)
  set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
  set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(BUILD_BENCHMARKS OFF CACHE BOOL "" FORCE)
  set(BUILD_DOCUMENTATION OFF CACHE BOOL "" FORCE)
  set(PROVIDE_UNINSTALL_TARGET OFF CACHE BOOL "" FORCE)
  set(EXPORT_BUILD_DIR ON CACHE BOOL "" FORCE)
  set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
  add_subdirectory("${MVSLAM_THIRD_PARTY}/ceres-solver" EXCLUDE_FROM_ALL)
  if(TARGET Ceres::ceres)
    set(MVSLAM_CERES_TARGET Ceres::ceres)
  else()
    set(MVSLAM_CERES_TARGET ceres)
  endif()
endif()

# ---- g2o (20241228_git, optional) -----------------------------------------------------------
if(MVSLAM_WITH_G2O)
  set(G2O_BUILD_APPS OFF CACHE BOOL "" FORCE)
  set(G2O_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(G2O_USE_OPENGL OFF CACHE BOOL "" FORCE)
  set(G2O_USE_CHOLMOD OFF CACHE BOOL "" FORCE)
  set(G2O_USE_CSPARSE OFF CACHE BOOL "" FORCE)
  set(G2O_USE_OPENMP OFF CACHE BOOL "" FORCE)
  set(G2O_BUILD_SLAM2D_TYPES OFF CACHE BOOL "" FORCE)
  set(G2O_BUILD_SLAM2D_ADDON_TYPES OFF CACHE BOOL "" FORCE)
  set(G2O_BUILD_DATA_TYPES OFF CACHE BOOL "" FORCE)
  set(G2O_BUILD_ICP_TYPES OFF CACHE BOOL "" FORCE)
  set(G2O_BUILD_SCLAM2D_TYPES OFF CACHE BOOL "" FORCE)
  set(G2O_BUILD_SLAM3D_ADDON_TYPES OFF CACHE BOOL "" FORCE)
  set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
  add_subdirectory("${MVSLAM_THIRD_PARTY}/g2o" EXCLUDE_FROM_ALL)
endif()

# ---- GTSAM (4.3a2, optional) ----------------------------------------------------------------
if(MVSLAM_WITH_GTSAM)
  set(GTSAM_USE_SYSTEM_EIGEN ON CACHE BOOL "" FORCE)
  set(GTSAM_WITH_TBB OFF CACHE BOOL "" FORCE)
  set(GTSAM_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(GTSAM_BUILD_EXAMPLES_ALWAYS OFF CACHE BOOL "" FORCE)
  set(GTSAM_BUILD_UNSTABLE OFF CACHE BOOL "" FORCE)
  set(GTSAM_BUILD_PYTHON OFF CACHE BOOL "" FORCE)
  set(GTSAM_INSTALL_MATLAB_TOOLBOX OFF CACHE BOOL "" FORCE)
  set(GTSAM_BUILD_WITH_MARCH_NATIVE OFF CACHE BOOL "" FORCE)
  set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
  add_subdirectory("${MVSLAM_THIRD_PARTY}/gtsam" EXCLUDE_FROM_ALL)
endif()
