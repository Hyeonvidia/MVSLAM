add_library(mvslam_api OBJECT config.cpp system.cpp)
add_library(mvslam::api ALIAS mvslam_api)
target_link_libraries(mvslam_api PUBLIC mvslam::core mvslam::map mvslam::pipelines PRIVATE yaml-cpp::yaml-cpp)
mvslam_set_warnings(mvslam_api)
