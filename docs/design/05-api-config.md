# 05 — 공개 API와 설정

> cuVSLAM 공개 API 원칙(`cuvslam_design_analysis.md` §4): 클래스 3개 수준의 얇은 표면, `Config`는 생성 시 1회 해소·검증, setter 없음, 튜닝 노브는 `internals`로 추방, 모순 설정은 throw. 여기에 VIGIL의 "공개 헤더에 서드파티 타입 비노출·호스트를 죽이지 않음"을 더한다.

## 1. 공개 타입 (`include/mvslam/api/types.hpp`, 전부 POD 또는 표준 컨테이너)
```cpp
namespace mvslam {
using Timestamp = std::int64_t;                 // ns
struct Quat { double w, x, y, z; };
struct Vec3 { double x, y, z; };
struct Pose { Quat q; Vec3 t; };                // T_wb: body → world
struct PoseState { Pose T_wb; Vec3 v_w; Vec3 bias_gyro; Vec3 bias_acc; };
enum class PixelFormat : std::uint8_t { Gray8, Rgb8, Bgr8, Depth16, DepthF32 };
struct ImageView { const std::uint8_t* data; int width, height, stride; PixelFormat format; };
struct FrameInput { Timestamp timestamp; std::vector<ImageView> images; /* rig 카메라 순 */ std::optional<ImageView> depth; };
struct ImuSample { Timestamp timestamp; Vec3 gyro; Vec3 acc; };
enum class TrackingState : std::uint8_t { NotInitialized, Ok, RecentlyLost, Lost };
struct TrackStats { int tracks, tracks3d, inliers; bool keyframe_inserted; double frontend_ms; };
struct TrackResult { Timestamp timestamp; TrackingState state; std::optional<PoseState> pose; std::optional<std::uint64_t> keyframe_id; TrackStats stats; };
}
```
- 이미지는 **소유하지 않는 뷰**다. `track()`이 반환하기 전까지만 유효하면 된다(내부에서 필요한 만큼 복사).
- 카메라/IMU 파라미터는 `Config` 안의 POD(`CameraConfig{model, intrinsics[], distortion[], T_bc}`)로 받는다.

## 2. `Config` (`include/mvslam/api/config.hpp`)
```cpp
struct Config {
  SensorMode sensor;            // Mono | Stereo | RgbD
  InertialMode inertial;        // None | Imu
  ExecutionMode execution;      // Realtime | Offline
  RigConfig rig;                // cameras[], T_bc, IMU calib(noise, walk, rate), body frame 정의
  FrontendConfig frontend;      // 02-frontend §4 표
  BackendConfig backend;        // 03-backend §6 표 (윈도우, 반복, 시간 상한)
  LoopConfig loop;              // enable, recognizer type, vocab path, thresholds
  MapConfig map;                // multi-map, culling, memory caps, load/save path
  LoggingConfig logging;        // level, sink
  static Config fromYaml(const std::string& path);   // 파싱 + validate(), 실패 시 ConfigError throw
  void validate() const;                             // 모순 검사: 예) Stereo인데 카메라 1대, Imu인데 IMU calib 없음, Offline인데 async 옵션 true
};
struct Internals;  // include/mvslam/api/internals.hpp — 전문가 노브(λ₀, δ, 반복 상수). Config::internals에 optional로 부착
```
- **불변**: `System`은 생성자에서 `Config`를 값으로 받아 보관한다. 런타임 변경은 없다(새 `System`을 만든다).
- **모드 이름**: `execution=Offline`이면 워커를 만들지 않고 모든 백엔드 작업을 `track()` 안에서 동기 실행하며 RNG 시드를 고정한다. `async_*` 류 하위 플래그를 Offline과 함께 켜면 `validate()`가 throw(cuVSLAM `Tracker::Mode`).
- 유효 설정 덤프: `System::effectiveConfigYaml()`(DM-VIO `usedSettings` 패턴) — 재현성.
- 설정 파일 예: `configs/euroc_stereo_imu.yaml`. 스키마 버전 키 `schema: 1`.

## 3. `System` (Facade, `include/mvslam/api/system.hpp`)
```cpp
class System {
 public:
  explicit System(Config cfg);                 // 파이프라인 합성 + 워커 기동. 실패는 throw(ConfigError, InitError)
  ~System();                                   // 워커 join(타임아웃), 미저장 맵 경고
  System(const System&) = delete;              // 복사 금지, 이동 허용
  void pushImu(const ImuSample&);              // 시간 단조 증가 필수, 호출자가 직렬화(cuVSLAM 계약)
  TrackResult track(const FrameInput&);        // 논블로킹(Realtime) / 동기(Offline). 실패 = state, 예외 아님
  MapService& mapService();                    // 저장/로드/재측위/이벤트
  Result<Pose> localize(const FrameInput& input);   // 무상태 단일 프레임 재측위(모든 맵), 추적 상태 불변 (P1)
  std::vector<PoseStamped> trajectory() const; // KF 보정 반영된 전체 프레임 궤적(T_wk·T_kb)
  Stats stats() const;                         // 타이밍·큐 길이·맵 크기
  std::string effectiveConfigYaml() const;
 private: struct Impl; std::unique_ptr<Impl> impl_;   // pimpl — ABI 안정, 서드파티 타입 은닉
};
```
- **에러 정책 3분류**(cuVSLAM): API 오용(잘못된 Config, 역행 타임스탬프) = 예외; 추적 실패 = `TrackResult.state`/`optional`; 비동기 작업(save/load/localize) = 콜백에 `Result<T>{ok, error_message}`. 코어는 `std::exit`·`abort`를 호출하지 않는다.
- 스레드 계약: `pushImu`·`track`은 **한 스레드에서** 호출한다(외부 직렬화). `mapService()`의 save/load/localize는 다른 스레드에서 호출 가능(커맨드 큐로 직렬화).
- Odometry 전용: `Config.loop.enable=false && map.multi_map=false`면 Loop 워커가 생성되지 않는다(cuVSLAM `OdometryOnly`).

## 4. `MapService`
```cpp
class MapService {
 public:
  void save(std::string path, std::function<void(Result<void>)> cb);   // 비동기(Realtime)/동기(Offline)
  void load(std::string path, std::function<void(Result<MapSummary>)> cb);
  void localize(std::optional<Pose> prior, std::function<void(Result<Pose>)> cb);  // 다음 track() 프레임들에서 전 맵 재측위를 시도(무장); 성공 시 그 맵으로 전환·추적 계속, 콜백 1회 (frontend.reloc_max_attempts 프레임 후 실패 보고). prior는 예약
  MapSnapshot snapshot() const;   // 뷰어/도구용 복사본: KF pose·타임스탬프, MP xyz (read lock 아래 O(크기))
  Subscription subscribe(EventHandler h);      // MapEvent variant(KeyframeInserted, PosesCorrected, …), 해제는 RAII
  MapSummary summary() const;                  // 맵 수, KF/MP 수, 활성 맵 id
};
```
- 명령은 `Command` 객체로 Loop 워커 큐에 들어간다(cuVSLAM `ICommand`; SLAM 워커가 KF와 같은 큐에서 순서대로 처리).

## 5. C ABI (후순위, `include/mvslam/c/mvslam_c.h`)
- `mvslam_handle*` opaque + 위 POD를 C 구조체로 미러링, 모든 함수 `mvslam_status_t` 반환, 콜백은 함수 포인터 + `void* user`. Swift(iOS)·Python(ctypes/nanobind) 바인딩의 기반. C++ API가 C ABI 위 래퍼가 아니라 **C ABI가 C++ pimpl 위의 얇은 래퍼**다 — 코어 구현은 하나.

## 6. 버전
- 라이브러리 `MVSLAM_VERSION`(semver) + 맵 포맷 버전 + 설정 스키마 버전을 `version.hpp`에서 노출. 공개 헤더 변경은 `docs/adr/`에 기록.
