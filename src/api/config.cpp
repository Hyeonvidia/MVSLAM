// Config parsing (yaml-cpp, private to this TU) and validation.
#include "mvslam/api/config.hpp"

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <sstream>

#include "mvslam/api/errors.hpp"

namespace mvslam {
namespace {

template <typename T>
T get(const YAML::Node& n, const char* key, const T& def) {
  if (!n || !n[key]) return def;
  try { return n[key].as<T>(); } catch (const YAML::Exception& e) { throw ConfigError(std::string("config key '") + key + "': " + e.what()); }
}
template <std::size_t N>
std::array<double, N> getArray(const YAML::Node& n, const char* key, const std::array<double, N>& def) {
  if (!n || !n[key]) return def;
  const auto v = n[key].as<std::vector<double>>();
  if (v.size() != N) throw ConfigError(std::string("config key '") + key + "' must have " + std::to_string(N) + " values");
  std::array<double, N> out{};
  std::copy(v.begin(), v.end(), out.begin());
  return out;
}
SensorMode parseSensor(const std::string& s) {
  if (s == "mono") return SensorMode::Mono;
  if (s == "stereo") return SensorMode::Stereo;
  if (s == "rgbd") return SensorMode::RgbD;
  throw ConfigError("sensor must be mono|stereo|rgbd, got '" + s + "'");
}
const char* sensorName(SensorMode m) { return m == SensorMode::Mono ? "mono" : m == SensorMode::Stereo ? "stereo" : "rgbd"; }

CameraConfig parseCamera(const YAML::Node& n) {
  CameraConfig c;
  const auto model = get<std::string>(n, "model", "pinhole");
  if (model == "pinhole") c.model = CameraModelType::Pinhole;
  else if (model == "kannala_brandt8") c.model = CameraModelType::KannalaBrandt8;
  else throw ConfigError("camera model must be pinhole|kannala_brandt8");
  c.width = get<int>(n, "width", 0);
  c.height = get<int>(n, "height", 0);
  c.intrinsics = getArray<4>(n, "intrinsics", {});
  c.distortion = get<std::vector<double>>(n, "distortion", {});
  c.T_bc = getArray<16>(n, "T_bc", c.T_bc);
  c.fps = get<double>(n, "fps", 20.0);
  return c;
}

Config parse(const YAML::Node& root) {
  Config c;
  c.schema = get<int>(root, "schema", 1);
  c.sensor = parseSensor(get<std::string>(root, "sensor", "mono"));
  const auto inertial = get<std::string>(root, "inertial", "none");
  c.inertial = inertial == "imu" ? InertialMode::Imu : InertialMode::None;
  const auto exec = get<std::string>(root, "execution", "realtime");
  c.execution = exec == "offline" ? ExecutionMode::Offline : ExecutionMode::Realtime;
  c.rng_seed = get<std::uint64_t>(root, "rng_seed", 0);

  const auto rig = root["rig"];
  if (rig && rig["cameras"]) for (const auto& cam : rig["cameras"]) c.rig.cameras.push_back(parseCamera(cam));
  if (rig && rig["imu"]) {
    const auto n = rig["imu"];
    ImuConfig i;
    i.rate_hz = get<double>(n, "rate_hz", i.rate_hz);
    i.noise_gyro = get<double>(n, "noise_gyro", i.noise_gyro);
    i.noise_acc = get<double>(n, "noise_acc", i.noise_acc);
    i.walk_gyro = get<double>(n, "walk_gyro", i.walk_gyro);
    i.walk_acc = get<double>(n, "walk_acc", i.walk_acc);
    i.T_bi = getArray<16>(n, "T_bi", i.T_bi);
    i.gravity = get<double>(n, "gravity", i.gravity);
    c.rig.imu = i;
  }
  if (rig) c.rig.stereo_baseline_m = get<double>(rig, "stereo_baseline_m", 0.0);

  const auto f = root["frontend"];
  auto& F = c.frontend;
#define MV_F(name) F.name = get<decltype(F.name)>(f, #name, F.name)
  MV_F(clahe); MV_F(clahe_clip); MV_F(klt_window); MV_F(klt_levels); MV_F(klt_fb_max_px); MV_F(klt_max_err);
  MV_F(p3p_survival_ratio); MV_F(outlier_parallax_gate_px); MV_F(ransac_iters); MV_F(ransac_err_px);
  MV_F(pose_huber_chi2); MV_F(pose_max_iters); MV_F(pose_max_time_ms);
  MV_F(kf_min_parallax_px); MV_F(kf_track_ratio); MV_F(kf_occupancy_ratio); MV_F(kf_stereo_force_s);
  MV_F(kf_imu_interval_s); MV_F(kf_imu_preinit_interval_s); MV_F(kf_close_tracked_min); MV_F(kf_close_new_min);
  MV_F(close_depth_factor); MV_F(detector_cell_px); MV_F(detector_subpix); MV_F(stereo_epipolar_px);
  MV_F(rgbd_virtual_baseline_m); MV_F(min_depth_m); MV_F(max_depth_m); MV_F(rgbd_depth_scale); MV_F(recently_lost_inertial_s);
  MV_F(recently_lost_visual_s); MV_F(new_map_min_keyframes); MV_F(reloc_min_bow_matches); MV_F(reloc_min_inliers); MV_F(reloc_max_attempts); MV_F(reloc_detector_cell_px); MV_F(reloc_max_hamming);
  MV_F(init_min_features); MV_F(min_3d_for_ok); MV_F(reinit_after_frames); MV_F(p3p_min_tracks); MV_F(mono_init_min_parallax_px); MV_F(mono_init_median_depth); MV_F(stereo_reproj_px);
#undef MV_F
  const auto b = root["backend"];
  auto& B = c.backend;
  const auto solver = get<std::string>(b, "solver", "ceres");
  B.solver = solver == "g2o" ? SolverBackend::G2o : solver == "gtsam" ? SolverBackend::Gtsam : SolverBackend::Ceres;
#define MV_B(name) B.name = get<decltype(B.name)>(b, #name, B.name)
  MV_B(lba_max_keyframes); MV_B(lba_min_covisibility); MV_B(lba_iters); MV_B(lba_max_time_s);
  MV_B(inertial_window); MV_B(inertial_window_large); MV_B(inertial_max_fixed_visual);
  MV_B(anchored_inverse_depth); MV_B(l2_refine_after_robust); MV_B(huber_mono_chi2); MV_B(huber_stereo_chi2);
  MV_B(kf_cull_redundancy); MV_B(kf_cull_redundancy_stereo_inertial); MV_B(kf_cull_min_other_observers); MV_B(imu_init_min_span_mono_s);
  MV_B(imu_init_min_span_stereo_s); MV_B(imu_init_min_keyframes); MV_B(imu_refine1_s); MV_B(imu_refine2_s); MV_B(solver_threads);
  MV_B(triangulation_min_parallax_deg); MV_B(triangulation_max_reproj_px); MV_B(mp_cull_min_observations); MV_B(mp_cull_grace_keyframes); MV_B(kf_culling);
  MV_B(local_match_radius_px); MV_B(local_match_max_hamming); MV_B(local_match_max_keyframes);
#undef MV_B
  const auto l = root["loop"];
  auto& L = c.loop;
  L.enable = get<bool>(l, "enable", L.enable);
  const auto rec = get<std::string>(l, "recognizer", "none");
  L.recognizer = rec == "dbow" ? PlaceRecognizerType::DBoW : rec == "incremental" ? PlaceRecognizerType::Incremental : PlaceRecognizerType::None;
  L.vocabulary_path = get<std::string>(l, "vocabulary_path", "");
  L.min_matches = get<int>(l, "min_matches", L.min_matches);
  L.min_inliers = get<int>(l, "min_inliers", L.min_inliers);
  L.loose_ba_threshold = get<double>(l, "loose_ba_threshold", L.loose_ba_threshold);
  L.global_ba = get<bool>(l, "global_ba", L.global_ba);
  L.pgo_iters = get<int>(l, "pgo_iters", L.pgo_iters);
  const auto m = root["map"];
  auto& M = c.map;
  M.multi_map = get<bool>(m, "multi_map", M.multi_map);
  M.max_keyframes_per_map = get<int>(m, "max_keyframes_per_map", M.max_keyframes_per_map);
  M.max_stored_maps = get<int>(m, "max_stored_maps", M.max_stored_maps);
  M.load_path = get<std::string>(m, "load_path", "");
  M.save_path = get<std::string>(m, "save_path", "");
  M.retention_s = get<double>(m, "retention_s", M.retention_s);
  const auto lg = root["logging"];
  c.logging.level = get<std::string>(lg, "level", "info");
  c.logging.to_stderr = get<bool>(lg, "to_stderr", true);
  c.logging.file = get<std::string>(lg, "file", "");
  return c;
}

}  // namespace

Config Config::fromYamlString(const std::string& yaml) {
  YAML::Node root;
  try { root = YAML::Load(yaml); } catch (const YAML::Exception& e) { throw ConfigError(std::string("yaml parse error: ") + e.what()); }
  Config c = parse(root);
  c.validate();
  return c;
}

Config Config::fromYaml(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw ConfigError("cannot open config file: " + path);
  std::stringstream ss;
  ss << in.rdbuf();
  return fromYamlString(ss.str());
}

void Config::validate() const {
  if (schema != 1) throw ConfigError("unsupported config schema " + std::to_string(schema));
  const std::size_t n = rig.cameras.size();
  if (n == 0) throw ConfigError("rig.cameras must contain at least one camera");
  if (sensor == SensorMode::Stereo && n < 2) throw ConfigError("sensor=stereo requires two cameras");
  if (sensor == SensorMode::Mono && n != 1) throw ConfigError("sensor=mono requires exactly one camera");
  if (sensor == SensorMode::RgbD && n != 1) throw ConfigError("sensor=rgbd requires exactly one (color) camera");
  if (inertial == InertialMode::Imu && !rig.imu) throw ConfigError("inertial=imu requires rig.imu calibration");
  for (const auto& c : rig.cameras) {
    if (c.width <= 0 || c.height <= 0) throw ConfigError("camera width/height must be positive");
    if (c.intrinsics[0] <= 0 || c.intrinsics[1] <= 0) throw ConfigError("camera fx/fy must be positive");
  }
  if (frontend.klt_levels < 0 || frontend.klt_window < 3) throw ConfigError("frontend.klt_levels/klt_window out of range");
  if (backend.lba_max_keyframes <= 0 || backend.inertial_window <= 0) throw ConfigError("backend window sizes must be positive");
  if (execution == ExecutionMode::Offline && backend.solver_threads != 1)
    throw ConfigError("execution=offline requires backend.solver_threads=1 (determinism)");
  if (loop.enable && loop.recognizer == PlaceRecognizerType::DBoW && loop.vocabulary_path.empty())
    throw ConfigError("loop.recognizer=dbow requires loop.vocabulary_path");
}

std::string Config::toYaml() const {
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "schema" << YAML::Value << schema;
  out << YAML::Key << "sensor" << YAML::Value << sensorName(sensor);
  out << YAML::Key << "inertial" << YAML::Value << (inertial == InertialMode::Imu ? "imu" : "none");
  out << YAML::Key << "execution" << YAML::Value << (execution == ExecutionMode::Offline ? "offline" : "realtime");
  out << YAML::Key << "rng_seed" << YAML::Value << rng_seed;
  out << YAML::Key << "rig" << YAML::Value << YAML::BeginMap << YAML::Key << "cameras" << YAML::Value << YAML::BeginSeq;
  for (const auto& c : rig.cameras) {
    out << YAML::BeginMap << YAML::Key << "width" << YAML::Value << c.width << YAML::Key << "height" << YAML::Value << c.height
        << YAML::Key << "intrinsics" << YAML::Value << YAML::Flow << std::vector<double>(c.intrinsics.begin(), c.intrinsics.end())
        << YAML::Key << "distortion" << YAML::Value << YAML::Flow << c.distortion
        << YAML::Key << "T_bc" << YAML::Value << YAML::Flow << std::vector<double>(c.T_bc.begin(), c.T_bc.end()) << YAML::EndMap;
  }
  out << YAML::EndSeq;
  if (rig.imu) {
    out << YAML::Key << "imu" << YAML::Value << YAML::BeginMap << YAML::Key << "rate_hz" << YAML::Value << rig.imu->rate_hz
        << YAML::Key << "noise_gyro" << YAML::Value << rig.imu->noise_gyro << YAML::Key << "noise_acc" << YAML::Value << rig.imu->noise_acc
        << YAML::Key << "walk_gyro" << YAML::Value << rig.imu->walk_gyro << YAML::Key << "walk_acc" << YAML::Value << rig.imu->walk_acc
        << YAML::Key << "gravity" << YAML::Value << rig.imu->gravity << YAML::EndMap;
  }
  out << YAML::EndMap;
  out << YAML::Key << "backend" << YAML::Value << YAML::BeginMap
      << YAML::Key << "solver" << YAML::Value << (backend.solver == SolverBackend::G2o ? "g2o" : backend.solver == SolverBackend::Gtsam ? "gtsam" : "ceres")
      << YAML::Key << "lba_max_keyframes" << YAML::Value << backend.lba_max_keyframes
      << YAML::Key << "inertial_window" << YAML::Value << backend.inertial_window
      << YAML::Key << "solver_threads" << YAML::Value << backend.solver_threads << YAML::EndMap;
  out << YAML::Key << "loop" << YAML::Value << YAML::BeginMap << YAML::Key << "enable" << YAML::Value << loop.enable << YAML::EndMap;
  out << YAML::Key << "map" << YAML::Value << YAML::BeginMap << YAML::Key << "multi_map" << YAML::Value << map.multi_map << YAML::EndMap;
  out << YAML::EndMap;
  return out.c_str();
}

}  // namespace mvslam
