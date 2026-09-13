# Rerun viewer sink: an adapter over the PUBLIC API only (System, MapService events/snapshots).
# Not part of libmvslam — apps and hosts opt in. Requires an installed Rerun C++ SDK (find_package).
add_library(mvslam_viz_rerun STATIC rerun_sink.cpp)
add_library(mvslam::viz_rerun ALIAS mvslam_viz_rerun)
target_include_directories(mvslam_viz_rerun PUBLIC "${PROJECT_SOURCE_DIR}/src")
target_link_libraries(mvslam_viz_rerun PUBLIC mvslam::mvslam rerun_sdk)
target_compile_definitions(mvslam_viz_rerun PUBLIC MVSLAM_HAVE_RERUN=1)
mvslam_set_warnings(mvslam_viz_rerun)
