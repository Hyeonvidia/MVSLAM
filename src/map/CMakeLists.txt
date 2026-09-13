add_library(mvslam_map OBJECT covisibility_graph.cpp map.cpp hbst_place_recognizer.cpp serialization/map_archive.cpp)
add_library(mvslam::map ALIAS mvslam_map)
target_link_libraries(mvslam_map PUBLIC mvslam::core mvslam::sensors mvslam::cereal)
mvslam_set_warnings(mvslam_map)
