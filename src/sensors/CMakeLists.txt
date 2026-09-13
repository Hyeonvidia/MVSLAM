add_library(mvslam_sensors OBJECT camera_model.cpp camera_rig.cpp imu/preintegration.cpp)
add_library(mvslam::sensors ALIAS mvslam_sensors)
target_link_libraries(mvslam_sensors PUBLIC mvslam::core)
mvslam_set_warnings(mvslam_sensors)
