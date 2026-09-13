#pragma once
// Immutable configuration tree. Built once (fromYaml / aggregate init), validated once (validate()),
// then owned by value by System. There are no setters (docs/design/05-api-config.md §2).
#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace mvslam {

enum class SensorMode : std::uint8_t { Mono, Stereo, RgbD };
enum class InertialMode : std::uint8_t { None, Imu };
enum class ExecutionMode : std::uint8_t { Realtime, Offline };
enum class CameraModelType : std::uint8_t { Pinhole, KannalaBrandt8 };
enum class SolverBackend : std::uint8_t { Ceres, G2o, Gtsam };
enum class PlaceRecognizerType : std::uint8_t { None, DBoW, Incremental };

struct CameraConfig {
  CameraModelType model = CameraModelType::Pinhole;
  int width = 0, height = 0;
  std::array<double, 4> intrinsics{};          // fx fy cx cy
  std::vector<double> distortion;              // pinhole: k1 k2 p1 p2 [k3]; KB8: k1..k4
  std::array<double, 16> T_bc{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};  // row-major 4x4, camera -> body
  double fps = 20.0;
};

struct ImuConfig {
  double rate_hz = 200.0;
  double noise_gyro = 1.7e-4, noise_acc = 2.0e-3;   // continuous-time densities
  double walk_gyro = 1.9e-5, walk_acc = 3.0e-3;
  std::array<double, 16> T_bi{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};  // imu -> body (identity: body == imu)
  double gravity = 9.81;
};

struct RigConfig {
  std::vector<CameraConfig> cameras;   // camera 0 defines the body frame when there is no IMU
  std::optional<ImuConfig> imu;
  double stereo_baseline_m = 0.0;      // derived for stereo when 0
};

struct FrontendConfig {
  bool clahe = true; double clahe_clip = 3.0;
  int klt_window = 9, klt_levels = 3; double klt_fb_max_px = 0.5, klt_max_err = 30.0;
  double p3p_survival_ratio = 0.33;
  double outlier_parallax_gate_px = 6.0; int ransac_iters = 100; double ransac_err_px = 3.0;
  double pose_huber_chi2 = 5.9915; int pose_max_iters = 5; double pose_max_time_ms = 5.0;
  double kf_min_parallax_px = 20.0, kf_track_ratio = 0.75, kf_occupancy_ratio = 0.5;
  double kf_stereo_force_s = 1.0, kf_imu_interval_s = 0.5, kf_imu_preinit_interval_s = 0.25;
  int kf_close_tracked_min = 100, kf_close_new_min = 70; double close_depth_factor = 60.0;
  int detector_cell_px = 35; bool detector_subpix = true;
  double stereo_epipolar_px = 2.0, rgbd_virtual_baseline_m = 0.08, min_depth_m = 0.1, max_depth_m = 40.0;
  double rgbd_depth_scale = 1.0 / 5000.0;  // Depth16 units -> meters (TUM: 5000/m)
  double recently_lost_inertial_s = 5.0, recently_lost_visual_s = 3.0; int new_map_min_keyframes = 10;
  int reloc_min_bow_matches = 15, reloc_min_inliers = 20;  // P3P-RANSAC + BA inliers (V101 tuning: 25-30 typical, precision unchanged)
  int reloc_max_attempts = 30;                              // MapService::localize(): frames tried before reporting failure
  int reloc_detector_cell_px = 20;                          // denser corners on the query image than for tracking (LOST-time only)
  int reloc_max_hamming = 60;                               // query corners are fresh detections, keyframe corners were KLT-tracked (~1 px drift)
  int reloc_ransac_iters = 1000;                            // P3P-RANSAC for relocalization (low inlier ratios after motion blur)
  int reloc_min_lost_frames = 5;                            // consecutive failed frames before relocalizing (KLT usually recovers a single bad frame by itself)
  bool inertial_pose_optimization = false;                  // frame pose from IMU factor + visual (MH01 stereo-inertial: 0.060/0.070 m vs 0.048 m with visual MotionOnlyBa + IMU prediction)
  int init_min_features = 100;          // stereo/rgbd: features required in the first keyframe; mono: matches
  int min_3d_for_ok = 15;               // 3-D tracks needed to leave Initializing / declare tracking ok
  int reinit_after_frames = 10;         // Initializing frames without 3-D tracks before restarting bootstrap
  int p3p_min_tracks = 8;
  double mono_init_min_parallax_px = 20.0;
  double mono_init_median_depth = 1.0;  // scale normalization of the mono bootstrap map
  double stereo_reproj_px = 3.0;        // stereo triangulation: max reprojection error in the right view
  double triangulationReprojPx() const { return stereo_reproj_px; }
};

struct BackendConfig {
  SolverBackend solver = SolverBackend::Ceres;
  int lba_max_keyframes = 20, lba_min_covisibility = 15, lba_iters = 10; double lba_max_time_s = 0.2;
  int inertial_window = 10, inertial_window_large = 25, inertial_max_fixed_visual = 200;
  bool inertial_lba = true;   // false: keep the visual covisibility LBA after IMU initialization (diagnostics)
  bool anchored_inverse_depth = true, l2_refine_after_robust = true;
  double huber_mono_chi2 = 5.991, huber_stereo_chi2 = 7.815;
  double kf_cull_redundancy = 0.9, kf_cull_redundancy_stereo_inertial = 0.5; int kf_cull_min_other_observers = 3;
  double imu_init_min_span_mono_s = 2.0, imu_init_min_span_stereo_s = 1.0; int imu_init_min_keyframes = 10;
  double imu_refine1_s = 5.0, imu_refine2_s = 15.0;
  int solver_threads = 1;
  double triangulation_min_parallax_deg = 1.0, triangulation_max_reproj_px = 3.0;
  int mp_cull_min_observations = 2, mp_cull_grace_keyframes = 3;
  bool kf_culling = false;  // ORB-SLAM redundancy rule; off for the KLT pipeline (MH01: 95% of keyframes culled, ATE 0.03 -> 0.26 m)
  double local_match_radius_px = 5.0; int local_match_max_hamming = 50, local_match_max_keyframes = 20;  // local-map re-observation
};

struct LoopConfig {
  bool enable = true;
  PlaceRecognizerType recognizer = PlaceRecognizerType::None;  // None: loop closing disabled until P3
  std::string vocabulary_path;
  int min_matches = 15, min_inliers = 30; double loose_ba_threshold = 0.02;
  bool global_ba = false; int pgo_iters = 20;
};

struct MapConfig {
  bool multi_map = true;
  int max_keyframes_per_map = 0;   // 0 = unlimited (culling controls growth)
  int max_stored_maps = 4;
  std::string load_path, save_path;
  double retention_s = 5.0;
};

struct LoggingConfig { std::string level = "info"; bool to_stderr = true; std::string file; };

struct Config {
  int schema = 1;
  SensorMode sensor = SensorMode::Mono;
  InertialMode inertial = InertialMode::None;
  ExecutionMode execution = ExecutionMode::Realtime;
  RigConfig rig;
  FrontendConfig frontend;
  BackendConfig backend;
  LoopConfig loop;
  MapConfig map;
  LoggingConfig logging;
  std::uint64_t rng_seed = 0;

  // Parses a YAML file (configs/*.yaml) and validates. Throws ConfigError on any problem.
  static Config fromYaml(const std::string& path);
  static Config fromYamlString(const std::string& yaml);
  // Rejects contradictory combinations (e.g. Stereo with one camera, Imu without imu calib).
  void validate() const;
  // Human-readable effective configuration (for run reproducibility logs).
  std::string toYaml() const;
};

}  // namespace mvslam
