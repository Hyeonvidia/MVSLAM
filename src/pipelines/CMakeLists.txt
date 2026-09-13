add_library(mvslam_pipelines OBJECT pipeline_factory.cpp)
add_library(mvslam::pipelines ALIAS mvslam_pipelines)
target_link_libraries(mvslam_pipelines PUBLIC mvslam::core mvslam::sensors mvslam::map mvslam::frontend mvslam::backend)
mvslam_set_warnings(mvslam_pipelines)
