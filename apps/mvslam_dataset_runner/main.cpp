// Dataset runner: EuRoC ASL layout (--euroc <mav0>) or KITTI odometry (--kitti <sequences/NN>). Drives
// System with images (+IMU) and writes a TUM trajectory. KITTI: intrinsics/baseline are read from the
// sequence's calib.txt (P0/P1) and override the config rig; timestamps come from times.txt.
// Usage: mvslam_dataset_runner --config <yaml> (--euroc <mav0> | --kitti <seq dir> [--kitti-calib calib.txt] | --tum <rgbd dir>) --out traj.txt
//        [--execution offline|realtime] [--save-map map.mvs] [--load-map map.mvs] [--localize-every N --localize-out loc.txt [--no-track 1]]
//        [--start-frame N] [--localize-arm 1]   (start mid-sequence and relocalize into the loaded map, then keep tracking)
//        [--pace F]   play at F x the dataset rate (sleep between frames by timestamp; 0 = as fast as possible) for live viewing
//        [--blackout START:DURATION]   camera outage: frames in [START, START+DURATION] s after the first frame are fed as black images
//                                      (the IMU keeps flowing) — inertial dead-reckoning / recovery test
//        [--rerun out.rrd | --rerun rerun+http://host.docker.internal:9876/proxy] [--rerun-image-every N]
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <thread>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "mvslam/api/system.hpp"
#ifdef MVSLAM_HAVE_RERUN
#include "platform/viz/rerun_sink.hpp"
#endif

namespace {
struct Entry { mvslam::Timestamp t; std::string file; };
std::vector<Entry> readCsv(const std::string& path) {
  std::vector<Entry> out;
  std::ifstream in(path);
  std::string line;
  while (std::getline(in, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();  // EuRoC csv files are CRLF
    if (line.empty() || line[0] == '#') continue;
    std::stringstream ss(line);
    std::string ts, name;
    std::getline(ss, ts, ',');
    std::getline(ss, name, ',');
    out.push_back({std::stoll(ts), name});
  }
  return out;
}
std::vector<mvslam::ImuSample> readImu(const std::string& path) {
  std::vector<mvslam::ImuSample> out;
  std::ifstream in(path);
  std::string line;
  while (std::getline(in, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line.empty() || line[0] == '#') continue;
    std::replace(line.begin(), line.end(), ',', ' ');
    std::stringstream ss(line);
    mvslam::ImuSample s;
    ss >> s.timestamp >> s.gyro.x >> s.gyro.y >> s.gyro.z >> s.acc.x >> s.acc.y >> s.acc.z;
    out.push_back(s);
  }
  return out;
}
// KITTI odometry sequence: image_0/%06d.png (+image_1), times.txt (seconds), calib.txt (P0..P3 rows).
struct KittiSequence { std::vector<Entry> left, right; };
KittiSequence readKitti(const std::string& dir, bool stereo) {
  KittiSequence s;
  std::ifstream in(dir + "/times.txt");
  std::string line;
  int i = 0;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    char name[16];
    std::snprintf(name, sizeof(name), "%06d.png", i++);
    const auto t = static_cast<mvslam::Timestamp>(std::llround(std::stod(line) * 1e9));
    s.left.push_back({t, name});
    if (stereo) s.right.push_back({t, name});
  }
  return s;
}
// Overrides the rig with the rectified KITTI calibration (cam0 = body; cam1 at +baseline along x).
void applyKittiCalib(mvslam::Config& cfg, const std::string& calib, int width, int height) {
  std::ifstream in(calib);
  std::string line;
  std::vector<double> P0, P1;
  while (std::getline(in, line)) {
    if (line.rfind("P0:", 0) != 0 && line.rfind("P1:", 0) != 0) continue;
    std::stringstream ss(line.substr(3));
    std::vector<double> v;
    for (double x; ss >> x;) v.push_back(x);
    (line[1] == '0' ? P0 : P1) = v;
  }
  if (P0.size() < 12 || P1.size() < 12) throw std::runtime_error("KITTI calib.txt: P0/P1 not found in " + calib);
  const double baseline = -P1[3] / P1[0];
  const std::size_t n = cfg.sensor == mvslam::SensorMode::Stereo ? 2 : 1;
  cfg.rig.cameras.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    auto& c = cfg.rig.cameras[i];
    c.model = mvslam::CameraModelType::Pinhole;
    c.width = width; c.height = height;
    c.intrinsics = {P0[0], P0[5], P0[2], P0[6]};
    c.distortion.clear();
    c.T_bc = {1, 0, 0, i == 1 ? baseline : 0.0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    c.fps = 10;
  }
  cfg.rig.stereo_baseline_m = n == 2 ? baseline : 0.0;
}
// TUM RGB-D: rgb.txt / depth.txt ("timestamp filename"), depth PNG 16-bit (5000 = 1 m), associated by nearest time.
struct TumSequence { std::vector<Entry> rgb, depth; };
std::vector<Entry> readTumList(const std::string& path) {
  std::vector<Entry> out;
  std::ifstream in(path);
  std::string line;
  while (std::getline(in, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line.empty() || line[0] == '#') continue;
    std::stringstream ss(line);
    double t; std::string file;
    if (!(ss >> t >> file)) continue;
    out.push_back({static_cast<mvslam::Timestamp>(std::llround(t * 1e9)), file});
  }
  return out;
}
TumSequence readTum(const std::string& dir, double max_dt_s = 0.02) {
  TumSequence s;
  const auto rgb = readTumList(dir + "/rgb.txt"), depth = readTumList(dir + "/depth.txt");
  std::size_t j = 0;
  for (const auto& r : rgb) {
    while (j + 1 < depth.size() && std::llabs(depth[j + 1].t - r.t) < std::llabs(depth[j].t - r.t)) ++j;
    if (depth.empty() || std::llabs(depth[j].t - r.t) > static_cast<mvslam::Timestamp>(max_dt_s * 1e9)) continue;
    s.rgb.push_back(r);
    s.depth.push_back(depth[j]);
  }
  return s;
}
}  // namespace

int main(int argc, char** argv) {
  std::string config, euroc, kitti, kitti_calib, tum, out = "trajectory.txt", execution, overlay_dir, rerun_target, save_map, load_map, localize_out;
  int overlay_every = 20, max_frames = 0, status_every = 200, status_from = 0, rerun_image_every = 5, localize_every = 0, start_frame = 0, blackout_frames = 0;
  bool no_track = false, localize_arm = false;
  double pace = 0, blackout_start = -1, blackout_duration = 0;
  std::string log_level;
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string k = argv[i], v = argv[i + 1];
    if (k == "--config") config = v; else if (k == "--euroc") euroc = v; else if (k == "--kitti") kitti = v; else if (k == "--kitti-calib") kitti_calib = v;
    else if (k == "--tum") tum = v; else if (k == "--save-map") save_map = v; else if (k == "--load-map") load_map = v;
    else if (k == "--localize-every") localize_every = std::stoi(v); else if (k == "--localize-out") localize_out = v;
    else if (k == "--no-track") { no_track = v == "1" || v == "true"; }
    else if (k == "--start-frame") start_frame = std::stoi(v); else if (k == "--localize-arm") { localize_arm = v == "1" || v == "true"; }
    else if (k == "--pace") pace = std::stod(v);
    else if (k == "--blackout") { const auto c = v.find(':'); blackout_start = std::stod(v.substr(0, c)); blackout_duration = c == std::string::npos ? 0 : std::stod(v.substr(c + 1)); }
    else if (k == "--out") out = v; else if (k == "--execution") execution = v;
    else if (k == "--overlay-dir") overlay_dir = v; else if (k == "--overlay-every") overlay_every = std::stoi(v);
    else if (k == "--max-frames") max_frames = std::stoi(v);
    else if (k == "--status-every") status_every = std::stoi(v); else if (k == "--status-from") status_from = std::stoi(v);
    else if (k == "--log-level") log_level = v;
    else if (k == "--rerun") rerun_target = v; else if (k == "--rerun-image-every") rerun_image_every = std::stoi(v);
  }
  const int sources = (euroc.empty() ? 0 : 1) + (kitti.empty() ? 0 : 1) + (tum.empty() ? 0 : 1);
  if (config.empty() || sources != 1) { std::cerr << "usage: --config <yaml> (--euroc <mav0> | --kitti <sequence dir> | --tum <rgbd dir>) [--out traj.txt]\n"; return 2; }
  try {
    auto cfg = mvslam::Config::fromYaml(config);
    if (execution == "offline") cfg.execution = mvslam::ExecutionMode::Offline;      // evaluation runs are deterministic
    else if (execution == "realtime") cfg.execution = mvslam::ExecutionMode::Realtime;
    else if (!execution.empty()) { std::cerr << "--execution must be offline|realtime\n"; return 2; }
    if (!log_level.empty()) cfg.logging.level = log_level;
    std::vector<Entry> cam0, cam1, depth_list;
    std::vector<mvslam::ImuSample> imu;
    std::string left_dir, right_dir, depth_dir;
    if (!tum.empty()) {
      auto seq = readTum(tum);
      cam0 = std::move(seq.rgb); depth_list = std::move(seq.depth);
      left_dir = tum + "/"; depth_dir = tum + "/";
      if (cam0.empty()) { std::cerr << "error: no associated rgb/depth pairs in " << tum << '\n'; return 1; }
      if (cfg.sensor != mvslam::SensorMode::RgbD) std::cerr << "warning: --tum with a non-rgbd config (depth ignored)\n";
    } else if (!euroc.empty()) {
      cam0 = readCsv(euroc + "/cam0/data.csv");
      if (cfg.sensor == mvslam::SensorMode::Stereo) cam1 = readCsv(euroc + "/cam1/data.csv");
      if (cfg.inertial == mvslam::InertialMode::Imu) imu = readImu(euroc + "/imu0/data.csv");
      left_dir = euroc + "/cam0/data/"; right_dir = euroc + "/cam1/data/";
    } else {
      auto seq = readKitti(kitti, cfg.sensor == mvslam::SensorMode::Stereo);
      cam0 = std::move(seq.left); cam1 = std::move(seq.right);
      left_dir = kitti + "/image_0/"; right_dir = kitti + "/image_1/";
      if (cam0.empty()) { std::cerr << "error: no frames in " << kitti << "/times.txt\n"; return 1; }
      const cv::Mat first = cv::imread(left_dir + cam0[0].file, cv::IMREAD_GRAYSCALE);
      if (first.empty()) { std::cerr << "error: cannot read " << left_dir + cam0[0].file << '\n'; return 1; }
      applyKittiCalib(cfg, kitti_calib.empty() ? kitti + "/calib.txt" : kitti_calib, first.cols, first.rows);
    }
    mvslam::System slam(cfg);
    if (!load_map.empty()) {
      bool ok = false;
      slam.mapService().load(load_map, [&](mvslam::Result<mvslam::MapSummary> r) {
        ok = r.ok;
        if (r.ok) std::cout << "loaded " << load_map << ": maps=" << r.value->maps << " keyframes=" << r.value->keyframes << " points=" << r.value->map_points << '\n';
        else std::cerr << "error: " << r.error << '\n';
      });
      if (!ok) return 1;
    }
    if (localize_arm) {
      slam.mapService().localize(std::nullopt, [](mvslam::Result<mvslam::Pose> r) {
        if (r.ok) std::cout << "relocalized into the loaded map at (" << r.value->t.x << ", " << r.value->t.y << ", " << r.value->t.z << ")\n";
        else std::cerr << "relocalization: " << r.error << '\n';
      });
    }
    std::ofstream loc_out;
    int loc_attempts = 0, loc_ok = 0;
    if (localize_every > 0) { loc_out.open(localize_out.empty() ? "localization.txt" : localize_out); loc_out << std::fixed << std::setprecision(9); }
#ifdef MVSLAM_HAVE_RERUN
    std::unique_ptr<mvslam::viz::RerunSink> viewer;
    if (!rerun_target.empty()) {
      mvslam::viz::RerunOptions ro;
      ro.target = rerun_target;
      ro.image_every = rerun_image_every;
      viewer = std::make_unique<mvslam::viz::RerunSink>(slam, ro);
    }
#else
    if (!rerun_target.empty()) { std::cerr << "error: built without MVSLAM_WITH_RERUN\n"; return 2; }
#endif
    std::size_t imu_i = 0;
    int keyframes = 0;
    const std::size_t n_frames = max_frames > 0 ? std::min<std::size_t>(cam0.size(), static_cast<std::size_t>(max_frames)) : cam0.size();
    const auto wall0 = std::chrono::steady_clock::now();
    const mvslam::Timestamp t0 = cam0.empty() ? 0 : cam0[static_cast<std::size_t>(std::max(0, start_frame))].t;
    for (std::size_t i = static_cast<std::size_t>(std::max(0, start_frame)); i < n_frames; ++i) {
      if (pace > 0) {  // live viewing: wait until the dataset time of this frame (scaled by 1/pace) has elapsed on the wall clock
        const auto due = wall0 + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(static_cast<double>(cam0[i].t - t0) * 1e-9 / pace));
        std::this_thread::sleep_until(due);
      }
      while (imu_i < imu.size() && imu[imu_i].timestamp <= cam0[i].t) slam.pushImu(imu[imu_i++]);
      cv::Mat l = cv::imread(left_dir + cam0[i].file, cv::IMREAD_GRAYSCALE);
      cv::Mat r;
      if (!cam1.empty() && i < cam1.size()) r = cv::imread(right_dir + cam1[i].file, cv::IMREAD_GRAYSCALE);
      if (blackout_start >= 0 && !l.empty()) {  // camera outage: black images, IMU still delivered
        const double rel = static_cast<double>(cam0[i].t - t0) * 1e-9;
        if (rel >= blackout_start && rel < blackout_start + blackout_duration) { l.setTo(0); if (!r.empty()) r.setTo(0); ++blackout_frames; }
      }
      if (l.empty()) continue;
      cv::Mat d;
      if (!depth_list.empty() && i < depth_list.size()) d = cv::imread(depth_dir + depth_list[i].file, cv::IMREAD_UNCHANGED);
      mvslam::FrameInput in;
      in.timestamp = cam0[i].t;
      in.images.push_back({l.data, l.cols, l.rows, static_cast<int>(l.step), mvslam::PixelFormat::Gray8});
      if (!r.empty()) in.images.push_back({r.data, r.cols, r.rows, static_cast<int>(r.step), mvslam::PixelFormat::Gray8});
      if (!d.empty() && d.type() == CV_16UC1) in.depth = mvslam::ImageView{d.data, d.cols, d.rows, static_cast<int>(d.step), mvslam::PixelFormat::Depth16};
      if (localize_every > 0 && i % static_cast<std::size_t>(localize_every) == 0) {
        const auto lr = slam.localize(in);
        ++loc_attempts;
        loc_out << static_cast<double>(in.timestamp) * 1e-9 << ' ' << (lr.ok ? 1 : 0);
        if (lr.ok) { ++loc_ok; const auto& P = *lr.value; loc_out << ' ' << P.t.x << ' ' << P.t.y << ' ' << P.t.z << ' ' << P.q.x << ' ' << P.q.y << ' ' << P.q.z << ' ' << P.q.w; }
        else loc_out << " 0 0 0 0 0 0 1";
        loc_out << '\n';
        if (no_track) continue;
      } else if (no_track) {
        continue;
      }
      const auto t_track0 = std::chrono::steady_clock::now();
      const auto res = slam.track(in);
      const double track_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_track0).count();
      if (res.stats.keyframe_inserted) ++keyframes;
#ifdef MVSLAM_HAVE_RERUN
      if (viewer) viewer->onFrame(in, res, track_ms);
#endif
      if (!overlay_dir.empty() && overlay_every > 0 && i % static_cast<std::size_t>(overlay_every) == 0) {
        cv::Mat vis;
        cv::cvtColor(l, vis, cv::COLOR_GRAY2BGR);
        for (const auto& t : res.debug_tracks)
          cv::circle(vis, cv::Point2f(t.u, t.v), 3, t.is3d ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
        cv::putText(vis, "f=" + std::to_string(i) + " st=" + std::to_string(static_cast<int>(res.state)) + " tr=" + std::to_string(res.stats.tracks) +
                    " 3d=" + std::to_string(res.stats.tracks3d) + " kf=" + std::to_string(keyframes), {8, 20}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 0, 255}, 1);
        cv::imwrite(overlay_dir + "/" + std::to_string(i) + ".png", vis);
      }
      if (status_every > 0 && static_cast<int>(i) >= status_from && i % static_cast<std::size_t>(status_every) == 0) {
        const auto st = slam.stats();
        std::cout << "frame " << i << " state=" << static_cast<int>(res.state) << " tracks=" << res.stats.tracks << " 3d=" << res.stats.tracks3d
                  << " inl=" << res.stats.inliers << " kfs=" << keyframes << " map(kf=" << st.map.keyframes << ",mp=" << st.map.map_points
                  << ") fe=" << st.frontend_ms_avg << "ms map=" << st.mapper_ms_avg << "ms lba=" << st.optimizer_ms_avg << "ms\n";
      }
    }
#ifdef MVSLAM_HAVE_RERUN
    if (viewer) viewer->finish();
#endif
    if (!save_map.empty()) {
      slam.mapService().save(save_map, [&](mvslam::Result<void> r) {
        if (r.ok) std::cout << "saved map to " << save_map << '\n'; else std::cerr << "error: " << r.error << '\n';
      });
    }
    if (localize_every > 0) std::cout << "localization: " << loc_ok << " / " << loc_attempts << " frames localized\n";
    if (blackout_frames > 0) std::cout << "blackout: " << blackout_frames << " frames fed as black images\n";
    std::ofstream of(out);
    of << std::fixed << std::setprecision(9);
    for (const auto& p : slam.trajectory())
      of << static_cast<double>(p.timestamp) * 1e-9 << ' ' << p.T_wb.t.x << ' ' << p.T_wb.t.y << ' ' << p.T_wb.t.z << ' '
         << p.T_wb.q.x << ' ' << p.T_wb.q.y << ' ' << p.T_wb.q.z << ' ' << p.T_wb.q.w << '\n';
    const auto st = slam.stats();
    std::cout << "frames: " << n_frames << " poses: " << slam.trajectory().size() << " keyframes: " << keyframes
              << " map(kf=" << st.map.keyframes << ",mp=" << st.map.map_points << ") frontend " << st.frontend_ms_avg << " ms/frame, mapper "
              << st.mapper_ms_avg << " ms/kf, lba " << st.optimizer_ms_avg << " ms/kf -> " << out << '\n';
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << '\n';
    return 1;
  }
  return 0;
}
