#include "map/serialization/map_archive.hpp"

#include <cereal/archives/portable_binary.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/vector.hpp>

#include <algorithm>
#include <fstream>

#include "mvslam/api/version.hpp"

namespace mvslam::map {

namespace {
constexpr std::uint32_t kMagic = 0x4D53564D;  // "MVSM"

// Poses go through Sophus' raw storage ([qx qy qz qw tx ty tz]) in both directions: constructing an
// SO3 from a quaternion re-normalizes it and moves the last bits, which would break the
// save -> load -> save byte identity.
template <typename Ar> void se3(Ar& ar, core::SE3& T) { double* d = T.data(); ar(d[0], d[1], d[2], d[3], d[4], d[5], d[6]); }
template <typename Ar> void so3(Ar& ar, core::SO3& R) { double* d = R.data(); ar(d[0], d[1], d[2], d[3]); }
template <typename Ar> void vec2(Ar& ar, core::Vec2& v) { double x = v.x(), y = v.y(); ar(x, y); v = core::Vec2(x, y); }
template <typename Ar> void vec3(Ar& ar, core::Vec3& v) { double x = v.x(), y = v.y(), z = v.z(); ar(x, y, z); v = core::Vec3(x, y, z); }
template <typename Ar> void vec6(Ar& ar, core::Vec6& v) { for (int i = 0; i < 6; ++i) { double x = v(i); ar(x); v(i) = x; } }
template <typename Ar, typename Tag> void id(Ar& ar, core::Id<Tag>& i) { std::uint64_t v = i.value; ar(v); i = core::Id<Tag>{v}; }

template <typename T> std::vector<T> sorted(std::vector<T> v) { std::sort(v.begin(), v.end()); return v; }

void writeKeyframe(cereal::PortableBinaryOutputArchive& ar, const KeyFrame& kf) {
  core::SE3 T = kf.T_wb();
  core::Vec3 v = kf.velocity();
  core::Vec6 b = kf.bias();
  std::uint64_t kid = kf.id().value;
  std::int64_t ts = kf.timestamp();
  ar(kid, ts);
  se3(ar, T); vec3(ar, v); vec6(ar, b);
  std::uint32_t n = static_cast<std::uint32_t>(kf.keypoints().size());
  ar(n);
  for (auto kp : kf.keypoints()) {
    vec2(ar, kp.uv_raw); vec2(ar, kp.uv);
    std::int32_t level = kp.level, cam = kp.cam;
    std::uint64_t track = kp.track.value;
    std::uint8_t has_right = kp.has_right ? 1 : 0, outlier = kp.outlier ? 1 : 0;
    ar(level, cam, track, kp.depth, kp.info, has_right);
    vec2(ar, kp.uv_right);
    ar(outlier);
  }
  ar(kf.descriptors());
  std::vector<std::uint64_t> slots;
  for (auto s : kf.mapPointSlots()) slots.push_back(s.value);
  ar(slots);
  std::uint64_t parent = kf.parent().value, prev = kf.prevKf().value, next = kf.nextKf().value;
  ar(parent, prev, next);
  std::vector<std::uint64_t> children, loops;
  for (auto c : kf.children()) children.push_back(c.value);
  for (auto l : kf.loopEdges()) loops.push_back(l.value);
  ar(children, loops);
}

std::shared_ptr<KeyFrame> readKeyframe(cereal::PortableBinaryInputArchive& ar) {
  std::uint64_t kid; std::int64_t ts;
  ar(kid, ts);
  core::SE3 T; core::Vec3 v; core::Vec6 b = core::Vec6::Zero();
  se3(ar, T); vec3(ar, v); vec6(ar, b);
  auto kf = std::make_shared<KeyFrame>(core::KeyframeId{kid}, ts, T);
  kf->setVelocity(v); kf->setBias(b);
  std::uint32_t n; ar(n);
  kf->keypoints().reserve(n);
  for (std::uint32_t i = 0; i < n; ++i) {
    Keypoint kp;
    vec2(ar, kp.uv_raw); vec2(ar, kp.uv);
    std::int32_t level, cam; std::uint64_t track; std::uint8_t has_right, outlier;
    ar(level, cam, track, kp.depth, kp.info, has_right);
    vec2(ar, kp.uv_right);
    ar(outlier);
    kp.level = level; kp.cam = cam; kp.track = core::TrackId{track}; kp.has_right = has_right != 0; kp.outlier = outlier != 0;
    kf->keypoints().push_back(kp);
  }
  ar(kf->descriptors());
  std::vector<std::uint64_t> slots; ar(slots);
  for (auto s : slots) kf->mapPointSlots().push_back(core::MapPointId{s});
  std::uint64_t parent, prev, next; ar(parent, prev, next);
  kf->setParent(core::KeyframeId{parent}); kf->setPrevKf(core::KeyframeId{prev}); kf->setNextKf(core::KeyframeId{next});
  std::vector<std::uint64_t> children, loops; ar(children, loops);
  for (auto c : children) kf->children().insert(core::KeyframeId{c});
  for (auto l : loops) kf->loopEdges().insert(core::KeyframeId{l});
  return kf;
}

void writeMapPoint(cereal::PortableBinaryOutputArchive& ar, const MapPoint& mp) {
  std::uint64_t mid = mp.id().value, anchor = mp.anchor().value;
  core::Vec3 X = mp.position();
  ar(mid); vec3(ar, X); ar(anchor);
  std::uint32_t n = static_cast<std::uint32_t>(mp.observations().size());
  ar(n);
  for (const auto& [kf, slot] : mp.observations()) { std::uint64_t k = kf.value; std::int32_t idx = slot.keypoint_index, cam = slot.cam; ar(k, idx, cam); }
  ar(mp.descriptor());
  std::int32_t visible = mp.visibleCount(), found = mp.foundCount();
  ar(visible, found);
}

std::shared_ptr<MapPoint> readMapPoint(cereal::PortableBinaryInputArchive& ar) {
  std::uint64_t mid, anchor; core::Vec3 X;
  ar(mid); vec3(ar, X); ar(anchor);
  auto mp = std::make_shared<MapPoint>(core::MapPointId{mid}, X, core::KeyframeId{anchor});
  std::uint32_t n; ar(n);
  for (std::uint32_t i = 0; i < n; ++i) { std::uint64_t k; std::int32_t idx, cam; ar(k, idx, cam); mp->addObservation(core::KeyframeId{k}, ObservationSlot{idx, cam}); }
  std::vector<std::uint8_t> d; ar(d); mp->setDescriptor(std::move(d));
  std::int32_t visible, found; ar(visible, found); mp->setCounters(visible, found);
  return mp;
}

void writeMap(cereal::PortableBinaryOutputArchive& ar, const Map& m) {
  std::uint64_t mid = m.id().value;
  ar(mid);
  std::vector<std::uint64_t> origins;
  for (auto o : m.origins()) origins.push_back(o.value);
  ar(origins);
  const auto& imu = m.imuState();
  std::uint8_t init = imu.initialized ? 1 : 0, r1 = imu.refined1 ? 1 : 0, r2 = imu.refined2 ? 1 : 0;
  core::SO3 R_wg = imu.R_wg;
  ar(init, r1, r2, imu.scale); so3(ar, R_wg);
  std::uint64_t last = m.lastKeyframe().value;
  ar(last);
  const auto kfs = m.keyframes();  // sorted by id
  std::uint32_t nk = static_cast<std::uint32_t>(kfs.size());
  ar(nk);
  for (const auto& kf : kfs) writeKeyframe(ar, *kf);
  auto mps = m.mapPoints();
  std::sort(mps.begin(), mps.end(), [](const auto& a, const auto& b) { return a->id() < b->id(); });
  std::uint32_t np = static_cast<std::uint32_t>(mps.size());
  ar(np);
  for (const auto& mp : mps) writeMapPoint(ar, *mp);
  // Covisibility: for every keyframe its neighbour list (ordered map -> deterministic).
  for (const auto& kf : kfs) {
    const auto& nb = m.covisibility().neighbours(kf->id());
    std::uint32_t nn = static_cast<std::uint32_t>(nb.size());
    ar(nn);
    for (const auto& [other, w] : nb) { std::uint64_t o = other.value; std::int32_t ww = w; ar(o, ww); }
  }
  std::vector<std::pair<std::uint64_t, std::uint64_t>> aliases;
  for (const auto& [t, mp] : m.trackAliases()) aliases.emplace_back(t.value, mp.value);
  std::sort(aliases.begin(), aliases.end());
  std::uint32_t na = static_cast<std::uint32_t>(aliases.size());
  ar(na);
  for (const auto& [t, mp] : aliases) ar(t, mp);
  std::vector<std::uint64_t> culled_ids;
  for (const auto& [k, _] : m.culledRecords()) culled_ids.push_back(k.value);
  std::sort(culled_ids.begin(), culled_ids.end());
  std::uint32_t nc = static_cast<std::uint32_t>(culled_ids.size());
  ar(nc);
  for (auto k : culled_ids) {
    auto rec = m.culledRecords().at(core::KeyframeId{k});
    std::uint64_t ref = rec.reference.value;
    ar(k, ref); se3(ar, rec.T_rc); se3(ar, rec.T_wc_at_cull);
  }
}

std::shared_ptr<Map> readMap(cereal::PortableBinaryInputArchive& ar) {
  std::uint64_t mid; ar(mid);
  auto m = std::make_shared<Map>(core::MapId{mid});
  std::vector<std::uint64_t> origins; ar(origins);
  std::uint8_t init, r1, r2; double scale; core::SO3 R_wg;
  ar(init, r1, r2, scale); so3(ar, R_wg);
  std::uint64_t last; ar(last);
  std::uint32_t nk; ar(nk);
  std::vector<std::shared_ptr<KeyFrame>> kfs;
  kfs.reserve(nk);
  for (std::uint32_t i = 0; i < nk; ++i) kfs.push_back(readKeyframe(ar));
  std::uint32_t np; ar(np);
  std::vector<std::shared_ptr<MapPoint>> mps;
  mps.reserve(np);
  for (std::uint32_t i = 0; i < np; ++i) mps.push_back(readMapPoint(ar));
  std::vector<std::vector<std::pair<std::uint64_t, std::int32_t>>> cov(nk);
  for (std::uint32_t i = 0; i < nk; ++i) {
    std::uint32_t nn; ar(nn);
    for (std::uint32_t j = 0; j < nn; ++j) { std::uint64_t o; std::int32_t w; ar(o, w); cov[i].emplace_back(o, w); }
  }
  std::uint32_t na; ar(na);
  std::vector<std::pair<std::uint64_t, std::uint64_t>> aliases(na);
  for (auto& [t, mp] : aliases) ar(t, mp);
  std::uint32_t nc; ar(nc);
  struct Culled { std::uint64_t k, ref; core::SE3 T_rc, T_wc; };
  std::vector<Culled> culled(nc);
  for (auto& c : culled) { ar(c.k, c.ref); se3(ar, c.T_rc); se3(ar, c.T_wc); }

  m->withWriteLock([&](Map& mm) {
    for (auto& kf : kfs) mm.addKeyFrame(kf);   // slots and links restored verbatim by readKeyframe
    for (auto& mp : mps) mm.addMapPoint(mp);   // observations restored verbatim by readMapPoint
    std::vector<core::KeyframeId> o;
    for (auto v : origins) o.push_back(core::KeyframeId{v});
    mm.setOrigins(o);
    auto& imu = mm.imuState();
    imu.initialized = init != 0; imu.refined1 = r1 != 0; imu.refined2 = r2 != 0; imu.scale = scale; imu.R_wg = R_wg;
    mm.setLastKeyframe(core::KeyframeId{last});
    // Covisibility is symmetric; setWeight per stored direction reproduces the adjacency exactly.
    for (std::uint32_t i = 0; i < nk; ++i)
      for (const auto& [o2, w] : cov[i]) mm.covisibility().setWeight(kfs[i]->id(), core::KeyframeId{o2}, w);
    for (const auto& [t, mp] : aliases) mm.addTrackAlias(core::TrackId{t}, core::MapPointId{mp});
    for (const auto& c : culled) mm.rememberCulled(core::KeyframeId{c.k}, Map::CulledRecord{core::KeyframeId{c.ref}, c.T_rc, c.T_wc});
    return 0;
  });
  return m;
}
}  // namespace

Result<void> saveMapStore(const MapStore& store, const std::string& path) {
  try {
    std::ofstream os(path, std::ios::binary);
    if (!os) return Result<void>::failure("save: cannot open " + path);
    cereal::PortableBinaryOutputArchive ar(os);
    std::uint32_t magic = kMagic, format = static_cast<std::uint32_t>(kMapFormatVersion);
    std::string version = kVersion;
    ar(magic, format, version);
    std::uint64_t next_kf = store.keyframeIds().peek(), next_mp = store.mapPointIds().peek(), next_track = store.trackIds().peek();
    ar(next_kf, next_mp, next_track);
    auto maps = store.all();
    maps.erase(std::remove_if(maps.begin(), maps.end(), [](const std::shared_ptr<Map>& m) { return m->withReadLock([](const Map& mm) { return mm.numKeyframes() == 0; }); }), maps.end());
    std::sort(maps.begin(), maps.end(), [](const auto& a, const auto& b) { return a->id() < b->id(); });
    std::uint32_t n = static_cast<std::uint32_t>(maps.size());
    ar(n);
    for (const auto& m : maps) m->withReadLock([&](const Map& mm) { writeMap(ar, mm); return 0; });
    return Result<void>::success();
  } catch (const std::exception& e) {
    return Result<void>::failure(std::string("save: ") + e.what());
  }
}

Result<MapSummary> loadMapStore(MapStore& store, const std::string& path) {
  try {
    std::ifstream is(path, std::ios::binary);
    if (!is) return Result<MapSummary>::failure("load: cannot open " + path);
    cereal::PortableBinaryInputArchive ar(is);
    std::uint32_t magic, format; std::string version;
    ar(magic, format, version);
    if (magic != kMagic) return Result<MapSummary>::failure("load: not an mvslam map file");
    if (format != static_cast<std::uint32_t>(kMapFormatVersion)) return Result<MapSummary>::failure("load: unsupported map format " + std::to_string(format));
    std::uint64_t next_kf, next_mp, next_track;
    ar(next_kf, next_mp, next_track);
    std::uint32_t n; ar(n);
    MapSummary s;
    std::uint64_t max_map = 0;
    for (std::uint32_t i = 0; i < n; ++i) {
      auto m = readMap(ar);
      max_map = std::max(max_map, m->id().value);
      m->withReadLock([&](const Map& mm) {
        s.keyframes += static_cast<int>(mm.numKeyframes());
        s.map_points += static_cast<int>(mm.numMapPoints());
        if (auto* pr = store.placeRecognizer())
          for (const auto& kf : mm.keyframes()) if (!kf->descriptors().empty()) pr->add(kf->id(), mm.id(), kf->descriptors(), 32);
        return 0;
      });
      store.addStored(m);
      ++s.maps;
    }
    store.keyframeIds().resume(std::max(store.keyframeIds().peek(), next_kf));
    store.mapPointIds().resume(std::max(store.mapPointIds().peek(), next_mp));
    store.trackIds().resume(std::max(store.trackIds().peek(), next_track));
    store.mapIds().resume(std::max(store.mapIds().peek(), max_map + 1));
    // The session's active map must not collide with loaded ids: recreate it when it is still empty.
    if (store.active()->withReadLock([](const Map& mm) { return mm.numKeyframes() == 0; })) store.resetActive();
    s.active_map = store.active()->id().value;
    return Result<MapSummary>::success(s);
  } catch (const std::exception& e) {
    return Result<MapSummary>::failure(std::string("load: ") + e.what());
  }
}

}  // namespace mvslam::map
