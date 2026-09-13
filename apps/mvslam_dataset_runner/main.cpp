// Dataset runner (EuRoC ASL layout). Drives System with images (+IMU) and writes a TUM trajectory.
// Usage: mvslam_dataset_runner --config configs/euroc_stereo_imu.yaml --euroc <mav0 dir> --out traj.txt [--execution offline|realtime]
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "mvslam/api/system.hpp"

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
}  // namespace

int main(int argc, char** argv) {
  std::string config, euroc, out = "trajectory.txt", execution;
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string k = argv[i], v = argv[i + 1];
    if (k == "--config") config = v; else if (k == "--euroc") euroc = v; else if (k == "--out") out = v; else if (k == "--execution") execution = v;
  }
  if (config.empty() || euroc.empty()) { std::cerr << "usage: --config <yaml> --euroc <mav0> [--out traj.txt]\n"; return 2; }
  try {
    auto cfg = mvslam::Config::fromYaml(config);
    if (execution == "offline") cfg.execution = mvslam::ExecutionMode::Offline;      // evaluation runs are deterministic
    else if (execution == "realtime") cfg.execution = mvslam::ExecutionMode::Realtime;
    else if (!execution.empty()) { std::cerr << "--execution must be offline|realtime\n"; return 2; }
    mvslam::System slam(cfg);
    const auto cam0 = readCsv(euroc + "/cam0/data.csv");
    const auto cam1 = cfg.sensor == mvslam::SensorMode::Stereo ? readCsv(euroc + "/cam1/data.csv") : std::vector<Entry>{};
    const auto imu = cfg.inertial == mvslam::InertialMode::Imu ? readImu(euroc + "/imu0/data.csv") : std::vector<mvslam::ImuSample>{};
    std::size_t imu_i = 0;
    for (std::size_t i = 0; i < cam0.size(); ++i) {
      while (imu_i < imu.size() && imu[imu_i].timestamp <= cam0[i].t) slam.pushImu(imu[imu_i++]);
      cv::Mat l = cv::imread(euroc + "/cam0/data/" + cam0[i].file, cv::IMREAD_GRAYSCALE);
      cv::Mat r;
      if (!cam1.empty() && i < cam1.size()) r = cv::imread(euroc + "/cam1/data/" + cam1[i].file, cv::IMREAD_GRAYSCALE);
      if (l.empty()) continue;
      mvslam::FrameInput in;
      in.timestamp = cam0[i].t;
      in.images.push_back({l.data, l.cols, l.rows, static_cast<int>(l.step), mvslam::PixelFormat::Gray8});
      if (!r.empty()) in.images.push_back({r.data, r.cols, r.rows, static_cast<int>(r.step), mvslam::PixelFormat::Gray8});
      slam.track(in);
    }
    std::ofstream of(out);
    for (const auto& p : slam.trajectory())
      of << static_cast<double>(p.timestamp) * 1e-9 << ' ' << p.T_wb.t.x << ' ' << p.T_wb.t.y << ' ' << p.T_wb.t.z << ' '
         << p.T_wb.q.x << ' ' << p.T_wb.q.y << ' ' << p.T_wb.q.z << ' ' << p.T_wb.q.w << '\n';
    std::cout << "frames: " << cam0.size() << " poses: " << slam.trajectory().size() << " -> " << out << '\n';
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << '\n';
    return 1;
  }
  return 0;
}
