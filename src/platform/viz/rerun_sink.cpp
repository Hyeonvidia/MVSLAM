#include "platform/viz/rerun_sink.hpp"

#include <rerun.hpp>

#include <atomic>
#include <cmath>
#include <deque>
#include <mutex>
#include <unordered_map>
#include <variant>

namespace mvslam::viz {

namespace {
// Minimal pose algebra on the public POD types (keeps the adapter free of Eigen/Sophus).
Quat mul(const Quat& a, const Quat& b) {
  return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
          a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}
Vec3 rotate(const Quat& q, const Vec3& v) {
  const Quat p{0, v.x, v.y, v.z};
  const Quat r = mul(mul(q, p), Quat{q.w, -q.x, -q.y, -q.z});
  return {r.x, r.y, r.z};
}
Pose compose(const Pose& a, const Pose& b) {  // a * b
  const Vec3 t = rotate(a.q, b.t);
  return {mul(a.q, b.q), {a.t.x + t.x, a.t.y + t.y, a.t.z + t.z}};
}
Pose poseFromMatrix(const std::array<double, 16>& m) {  // row-major 4x4 -> Pose (rotation via quaternion extraction)
  const double r00 = m[0], r01 = m[1], r02 = m[2], r10 = m[4], r11 = m[5], r12 = m[6], r20 = m[8], r21 = m[9], r22 = m[10];
  Quat q;
  const double tr = r00 + r11 + r22;
  if (tr > 0) { const double s = std::sqrt(tr + 1.0) * 2; q.w = 0.25 * s; q.x = (r21 - r12) / s; q.y = (r02 - r20) / s; q.z = (r10 - r01) / s; }
  else if (r00 > r11 && r00 > r22) { const double s = std::sqrt(1.0 + r00 - r11 - r22) * 2; q.w = (r21 - r12) / s; q.x = 0.25 * s; q.y = (r01 + r10) / s; q.z = (r02 + r20) / s; }
  else if (r11 > r22) { const double s = std::sqrt(1.0 + r11 - r00 - r22) * 2; q.w = (r02 - r20) / s; q.x = (r01 + r10) / s; q.y = 0.25 * s; q.z = (r12 + r21) / s; }
  else { const double s = std::sqrt(1.0 + r22 - r00 - r11) * 2; q.w = (r10 - r01) / s; q.x = (r02 + r20) / s; q.y = (r12 + r21) / s; q.z = 0.25 * s; }
  return {q, {m[3], m[7], m[11]}};
}
// The visualization up-axis: the world frame is the first body frame, which is not gravity aligned in
// general (EuRoC's IMU frame has +x up, camera-only rigs have -y up). Camera 0 looks down its +z with
// +y pointing down, so "up" in the body frame is -R_bc * e_y; the dominant axis picks the ViewCoordinates.
rerun::ViewCoordinates worldViewCoordinates(const Pose& T_bc) {
  const Vec3 up = rotate(T_bc.q, Vec3{0, -1, 0});
  const double ax = std::abs(up.x), ay = std::abs(up.y), az = std::abs(up.z);
  if (ax >= ay && ax >= az) return up.x > 0 ? rerun::ViewCoordinates::RIGHT_HAND_X_UP : rerun::ViewCoordinates::RIGHT_HAND_X_DOWN;
  if (ay >= az) return up.y > 0 ? rerun::ViewCoordinates::RIGHT_HAND_Y_UP : rerun::ViewCoordinates::RIGHT_HAND_Y_DOWN;
  return up.z > 0 ? rerun::ViewCoordinates::RIGHT_HAND_Z_UP : rerun::ViewCoordinates::RIGHT_HAND_Z_DOWN;
}
rerun::Transform3D toTransform(const Pose& T) {
  return rerun::Transform3D(rerun::components::Translation3D(static_cast<float>(T.t.x), static_cast<float>(T.t.y), static_cast<float>(T.t.z)),
                            rerun::Rotation3D(rerun::Quaternion::from_xyzw(static_cast<float>(T.q.x), static_cast<float>(T.q.y),
                                                                            static_cast<float>(T.q.z), static_cast<float>(T.q.w))));
}
const rerun::Color kColor3d(40, 220, 60), kColor2d(240, 200, 30), kColorTraj(230, 60, 60), kColorKf(70, 130, 255), kColorPoints(200, 200, 200);
}  // namespace

struct RerunSink::Impl {
  Impl(System& s, RerunOptions o) : sys(s), opt(std::move(o)), rec(opt.application_id) {
    if (opt.target == "spawn") rec.spawn().exit_on_failure();
    else if (opt.target.rfind("rerun+", 0) == 0) rec.connect_grpc(opt.target).exit_on_failure();
    else rec.save(opt.target).exit_on_failure();
    const auto& cam = sys.config().rig.cameras.at(0);
    fx = static_cast<float>(cam.intrinsics[0]); fy = static_cast<float>(cam.intrinsics[1]);
    width = cam.width; height = cam.height;
    T_bc = poseFromMatrix(cam.T_bc);
    rec.log_static("world", worldViewCoordinates(T_bc));
    rec.log_static("stats/frontend/ms", rerun::SeriesLines().with_names("frontend track() ms").with_colors(kColor3d));
    rec.log_static("stats/frontend/tracks3d", rerun::SeriesLines().with_names("3-D tracks").with_colors(kColor2d));
    rec.log_static("stats/backend/lba_ms", rerun::SeriesLines().with_names("local BA ms/KF (avg)").with_colors(kColorTraj));
    rec.log_static("stats/backend/mapper_ms", rerun::SeriesLines().with_names("mapper ms/KF (avg)").with_colors(kColorKf));
    rec.log("events", rerun::TextLog("mvslam started: " + std::to_string(width) + "x" + std::to_string(height) +
                                     " | image view: green = 3-D tracks (map points), yellow = new 2-D tracks (waiting for triangulation), tails = KLT motion over the last 10 frames").with_level(rerun::TextLogLevel::Info));
    rec.log_static("world/cam/image", pinhole().with_image_plane_distance(0.2f));
    sub = sys.mapService().subscribe([this](const MapEvent& e) { onEvent(e); });
  }
  rerun::Pinhole pinhole() const {
    return rerun::Pinhole::from_focal_length_and_resolution({fx, fy}, {static_cast<float>(width), static_cast<float>(height)})
        .with_camera_xyz(rerun::components::ViewCoordinates::RDF);
  }
  void setTime(Timestamp t, std::uint64_t frame) const {
    rec.set_time_timestamp_nanos_since_epoch("capture", t);
    rec.set_time_sequence("frame", static_cast<std::int64_t>(frame));
  }
  void logPoints() {
    const auto snap = sys.mapService().snapshot();
    std::vector<rerun::Position3D> pts;
    pts.reserve(snap.points.size());
    for (const auto& p : snap.points) pts.emplace_back(p.x, p.y, p.z);
    rec.log("world/points", rerun::Points3D(pts).with_colors(kColorPoints).with_radii(opt.point_radius_m));
    // Keyframe poses move with every LBA: refresh all frustums from the snapshot.
    for (const auto& kf : snap.keyframes) logKeyframe(kf.id, kf.T_wb);
  }
  void logKeyframe(std::uint64_t id, const Pose& T_wb) {
    const std::string path = "world/kf/" + std::to_string(id);
    rec.log(path, toTransform(compose(T_wb, T_bc)));
    rec.log(path, pinhole().with_image_plane_distance(0.08f).with_color(kColorKf));
  }
  void logTrajectory() {
    const auto traj = sys.trajectory();
    if (traj.size() < 2) return;
    std::vector<rerun::Position3D> strip;
    strip.reserve(traj.size());
    for (const auto& p : traj) strip.emplace_back(static_cast<float>(p.T_wb.t.x), static_cast<float>(p.T_wb.t.y), static_cast<float>(p.T_wb.t.z));
    rec.log("world/trajectory", rerun::LineStrips3D(rerun::LineStrip3D(strip)).with_colors(kColorTraj).with_radii(0.004f));
  }
  void onEvent(const MapEvent& e) {
    std::lock_guard<std::mutex> lk(mu);
    setTime(last_t.load(), last_frame.load());
    if (const auto* ki = std::get_if<KeyframeInsertedEvent>(&e)) {
      logKeyframe(ki->keyframe_id, ki->T_wb);
      const auto st = sys.stats();
      rec.log("stats/backend/mapper_ms", rerun::Scalars(st.mapper_ms_avg));
      rec.log("stats/backend/lba_ms", rerun::Scalars(st.optimizer_ms_avg));
      rec.log("events", rerun::TextLog("mapper: keyframe " + std::to_string(ki->keyframe_id) + " inserted (map: " + std::to_string(st.map.keyframes) + " KF, " +
                                       std::to_string(st.map.map_points) + " points)").with_level(rerun::TextLogLevel::Info));
      if (opt.points_every_keyframes > 0 && ++keyframes % opt.points_every_keyframes == 0) logPoints();
    } else if (const auto* mu = std::get_if<MapPointsUpdatedEvent>(&e)) {
      if (++lba_updates % 10 == 0) rec.log("events", rerun::TextLog("optimizer: local BA applied (map epoch " + std::to_string(mu->epoch) + ")").with_level(rerun::TextLogLevel::Debug));
    } else if (const auto* kr = std::get_if<KeyframeRemovedEvent>(&e)) {
      rec.log("world/kf/" + std::to_string(kr->keyframe_id), rerun::Clear::FLAT);
    } else if (std::get_if<MapResetEvent>(&e)) {
      rec.log("world/kf", rerun::Clear::RECURSIVE);
      rec.log("world/points", rerun::Clear::FLAT);
      rec.log("events", rerun::TextLog("map reset").with_level(rerun::TextLogLevel::Warning));
    }
  }
  void onFrame(const FrameInput& in, const TrackResult& r, double frame_ms) {
    std::lock_guard<std::mutex> lk(mu);
    ++frames;
    last_t.store(in.timestamp);
    last_frame.store(frames);
    setTime(in.timestamp, frames);
    if (r.pose) rec.log("world/cam", toTransform(compose(r.pose->T_wb, T_bc)));
    for (const auto& t : r.debug_tracks) {  // per-track position history for the KLT tails
      auto& tr = trails[t.id];
      tr.pts.emplace_back(t.u, t.v);
      if (tr.pts.size() > kTrailLen) tr.pts.pop_front();
      tr.last_seen = frames;
    }
    for (auto it = trails.begin(); it != trails.end();) it = it->second.last_seen == frames ? std::next(it) : trails.erase(it);
    if (frame_ms >= 0) rec.log("stats/frontend/ms", rerun::Scalars(frame_ms));
    rec.log("stats/frontend/tracks3d", rerun::Scalars(static_cast<double>(r.stats.tracks3d)));
    if (r.state != last_state) {
      static const char* kNames[] = {"NotInitialized", "Ok", "RecentlyLost", "Lost"};
      rec.log("events", rerun::TextLog(std::string("frontend: state -> ") + kNames[static_cast<int>(r.state) & 3]).with_level(r.state == TrackingState::Ok ? rerun::TextLogLevel::Info : rerun::TextLogLevel::Warning));
      last_state = r.state;
    }
    if (r.keyframe_id) rec.log("events", rerun::TextLog("frontend: keyframe " + std::to_string(*r.keyframe_id) + " requested (" + std::to_string(r.stats.tracks) + " tracks, " + std::to_string(r.stats.tracks3d) + " 3-D)").with_level(rerun::TextLogLevel::Debug));
    if (opt.image_every > 0 && frames % static_cast<std::uint64_t>(opt.image_every) == 0 && !in.images.empty()) {
      const auto& im = in.images[0];
      if (im.format == PixelFormat::Gray8) {
        std::vector<std::uint8_t> bytes(static_cast<std::size_t>(im.width) * static_cast<std::size_t>(im.height));
        for (int y = 0; y < im.height; ++y)
          std::copy(im.data + static_cast<std::size_t>(y) * static_cast<std::size_t>(im.stride), im.data + static_cast<std::size_t>(y) * static_cast<std::size_t>(im.stride) + im.width,
                    bytes.begin() + static_cast<std::ptrdiff_t>(y) * im.width);
        rec.log("world/cam/image", rerun::Image::from_grayscale8(bytes, {static_cast<uint32_t>(im.width), static_cast<uint32_t>(im.height)}));
      }
      // KLT tracks: green = tracks that already have a map point (3-D), yellow = new 2-D tracks waiting
      // for triangulation; the tails show where each track was over the last `trail_len` frames.
      std::vector<rerun::Position2D> p3, p2;
      std::vector<rerun::LineStrip2D> strips;
      std::vector<rerun::Color> strip_colors;
      for (const auto& t : r.debug_tracks) {
        (t.is3d ? p3 : p2).emplace_back(t.u, t.v);
        auto it = trails.find(t.id);
        if (it == trails.end() || it->second.pts.size() < 2) continue;
        strips.emplace_back(std::vector<rerun::Position2D>(it->second.pts.begin(), it->second.pts.end()));
        strip_colors.push_back(t.is3d ? rerun::Color(40, 220, 60, 160) : rerun::Color(240, 200, 30, 160));
      }
      rec.log("world/cam/image/tracks/3d_map_points", rerun::Points2D(p3).with_colors(kColor3d).with_radii(2.5f));
      rec.log("world/cam/image/tracks/2d_new", rerun::Points2D(p2).with_colors(kColor2d).with_radii(2.0f));
      rec.log("world/cam/image/tracks/klt_flow", rerun::LineStrips2D(strips).with_colors(strip_colors).with_radii(0.7f));
    }
    if (opt.trajectory_every > 0 && frames % static_cast<std::uint64_t>(opt.trajectory_every) == 0) logTrajectory();
  }
  void finish() {
    std::lock_guard<std::mutex> lk(mu);
    if (finished) return;
    finished = true;
    setTime(last_t.load(), last_frame.load());
    logPoints();
    logTrajectory();
    static_cast<void>(rec.flush_blocking());
  }

  System& sys;
  RerunOptions opt;
  rerun::RecordingStream rec;
  Subscription sub;
  std::mutex mu;
  float fx = 0, fy = 0;
  int width = 0, height = 0;
  Pose T_bc;
  static constexpr std::size_t kTrailLen = 10;
  struct Trail { std::deque<rerun::Position2D> pts; std::uint64_t last_seen = 0; };
  std::unordered_map<std::uint64_t, Trail> trails;
  std::uint64_t frames = 0;
  int keyframes = 0, lba_updates = 0;
  TrackingState last_state = TrackingState::NotInitialized;
  std::atomic<Timestamp> last_t{0};
  std::atomic<std::uint64_t> last_frame{0};
  bool finished = false;
};

RerunSink::RerunSink(System& system, RerunOptions options) : impl_(std::make_unique<Impl>(system, std::move(options))) {}
RerunSink::~RerunSink() { if (impl_) impl_->finish(); }
void RerunSink::onFrame(const FrameInput& input, const TrackResult& result, double frame_ms) { impl_->onFrame(input, result, frame_ms); }
void RerunSink::finish() { impl_->finish(); }

}  // namespace mvslam::viz
