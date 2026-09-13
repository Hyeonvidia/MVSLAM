# MVSLAM — Modular multi-mode Visual SLAM (C++20)

mono / stereo / RGB-D × (±IMU) 모드를 하나의 코드베이스로 지원하는 멀티플랫폼 Visual SLAM 라이브러리.
방법론은 OV²SLAM(KLT 프론트엔드·실시간 장치)과 ORB-SLAM3(백엔드·맵 계층·VI 정식화), 아키텍처는 cuVSLAM(명시적 API·비동기 워커·결정적 오프라인 모드·의존성 고정)에서 가져왔다.
**현재 상태: 설계 + 스켈레톤(P-skel).** 알고리즘 구현은 `docs/design/09-roadmap.md`의 P0부터.

## 문서
| 문서 | 내용 |
|---|---|
| [docs/design/00-overview.md](docs/design/00-overview.md) | 목표, 설계 입력, 10대 원칙, 관련 프로젝트, 라이선스, 용어 |
| [01-architecture.md](docs/design/01-architecture.md) | 계층·모듈·의존 규칙, 모드 합성(Factory), 데이터 흐름, 디렉터리 |
| [02-frontend.md](docs/design/02-frontend.md) | KLT 추적, 자세 추정, 상태기계, KF 정책, 초기화, IMU 예측, 파라미터 |
| [03-backend.md](docs/design/03-backend.md) | 그래프 추상, factor 목록, 솔버 어댑터, marginalization, LBA, IMU 초기화, 루프 클로징 |
| [04-map.md](docs/design/04-map.md) | id·소유권, 다중 맵, 락 규약, 이벤트, 직렬화, 메모리 상한 |
| [05-api-config.md](docs/design/05-api-config.md) | 공개 API, Config/validate, 에러 정책, C ABI |
| [06-concurrency.md](docs/design/06-concurrency.md) | 스레드·큐·선점·역압, Realtime/Offline, 결정성 |
| [07-build-platforms.md](docs/design/07-build-platforms.md) | 서브모듈 핀, CMake 옵션/프리셋, 플랫폼, CI, 코드 규약 |
| [08-design-patterns.md](docs/design/08-design-patterns.md) | 패턴 카탈로그와 반면교사 |
| [09-roadmap.md](docs/design/09-roadmap.md) | P0~P5 단계와 게이트 |
| [10-traceability.md](docs/design/10-traceability.md) | 결정 ↔ 분석 근거 |
| [docs/adr/](docs/adr/) | 아키텍처 결정 기록 |

## 빌드 (도커 전용)
호스트에서 직접 빌드하지 않는다. 빌드 트리는 도커 볼륨(`/workspace/build`)에만 생긴다.
```bash
git clone --recurse-submodules --shallow-submodules <repo> MVSLAM   # 또는 scripts/bootstrap.sh
scripts/docker.sh image          # 이미지 빌드 (ubuntu:24.04 툴체인 + 시스템 OpenCV)
scripts/docker.sh build          # docker-release: configure + build + ctest
scripts/docker.sh run configs/euroc_stereo_imu.yaml EuRoC/MH01 docker-release /results/MH01_traj.txt --execution offline
                                 # ../Datasets 를 /datasets 로 읽기 전용 마운트, 결과는 results/ (--execution offline = 결정적 평가 모드)
scripts/docker.sh eval EuRoC/MH01 /results/MH01_traj.txt            # ATE(SE3; mono는 --sim3)
scripts/docker.sh kitti configs/kitti_stereo.yaml 00 docker-release /results/kitti_00.txt --execution offline
scripts/docker.sh eval-kitti 00 /results/kitti_00.txt
scripts/docker.sh tsan           # ThreadSanitizer 빌드/테스트
scripts/docker.sh shell          # 컨테이너 셸
```
- 요구: Docker(compose v2). 이미지 안: CMake 3.28, gcc-13/clang, Ninja, ccache, OpenCV 4.6.
- 프리셋: `docker-release` / `docker-full`(g2o+GTSAM 어댑터) / `docker-asan` / `docker-tsan`. `macos-*`·`ios-*`·`jetson-*` 프리셋은 플랫폼 이식 검증용.
- 옵션: `MVSLAM_WITH_G2O`, `MVSLAM_WITH_GTSAM`, `MVSLAM_SANITIZER=asan|tsan`, `MVSLAM_SHARED`, `MVSLAM_OPENCV_SOURCE=auto|system|submodule|none`.

## P0 결과 (Offline, docker-release, Apple M4 컨테이너, 2026-09-14)
| 시퀀스 | 모드 | ATE RMSE | 프론트엔드 | 비고 |
|---|---|---|---|---|
| EuRoC MH01 | stereo | **0.032 m** (SE3) | 3.4 ms/frame | 게이트 ≤ 0.06 m 달성, 2회 실행 궤적 SHA 동일 |
| EuRoC MH01 | mono | 0.142 m (Sim3, scale 3.70) | 2.7 ms/frame | 두 시점 H/E 초기화, 루프 클로징 없음 |
| KITTI 00 | stereo | 4.99 m (SE3, 3.7 km) | 4.9 ms/frame | 루프 클로징 없음(P3) |

## P1 결과 (EuRoC V1_01 stereo, Offline, 2026-09-14)
| 항목 | 결과 |
|---|---|
| 맵 저장 → 로드 → 저장 | 바이트 동일 (21 MB, 535 KF / 15.6k MP) |
| 무상태 재측위 `System::localize` (20프레임마다 146회) | 146회 성공, 5 cm/3° 안 99.3%, 위치 중앙 오차 4.6 mm |
| 로드 맵 재측위 후 연속 추적 (프레임 1000부터) | ATE 0.050 m |
| TUM fr1_desk RGB-D | 데이터셋 미보유 — `--tum` 로더·`eval-tum` 준비됨 |

러너 옵션: 입력 `--euroc <mav0>` / `--kitti <sequences/NN> [--kitti-calib calib.txt]`(P0/P1로 rig 자동 설정) / `--tum <rgbd dir>`; `--out`, `--execution offline|realtime`, `--max-frames N`, `--start-frame N`, `--status-every N`, `--overlay-dir DIR [--overlay-every N]`(트랙 오버레이 PNG), `--log-level debug`; 맵 `--save-map map.mvs`, `--load-map map.mvs`, `--localize-arm 1`(로드 맵에 재측위 후 추적 계속), `--localize-every N --localize-out loc.txt [--no-track 1]`(무상태 재측위 평가 → `scripts/docker.sh eval-loc`); 뷰어 `--rerun out.rrd | rerun+http://host.docker.internal:9876/proxy [--rerun-image-every N]`.

## 뷰어 (Rerun)
코어는 뷰어를 모른다. `src/platform/viz/rerun_sink`가 공개 API(이벤트·스냅샷)만으로 카메라·이미지·트랙·KF 프러스텀·맵 포인트·궤적을 Rerun에 기록한다([ADR-0011](docs/adr/0011-rerun-viewer.md)).
```bash
pip install rerun-sdk==0.37.2                     # 호스트 뷰어 (SDK와 같은 버전)
scripts/docker.sh run configs/euroc_stereo_imu.yaml EuRoC/MH01 docker-release /results/MH01.txt --execution offline --rerun /results/MH01.rrd
rerun results/MH01.rrd                            # 기록 재생
rerun &  # 실시간: 호스트 뷰어를 띄운 뒤
scripts/docker.sh run configs/euroc_stereo_imu.yaml EuRoC/MH01 docker-release /results/MH01.txt --rerun rerun+http://host.docker.internal:9876/proxy
```

## 사용 예
```cpp
auto cfg = mvslam::Config::fromYaml("configs/euroc_stereo_imu.yaml");  // 모순 설정은 여기서 throw
mvslam::System slam(cfg);
slam.pushImu(sample);                          // 시간 단조 증가
auto r = slam.track(frame);                    // 논블로킹(Realtime) / 동기(Offline)
slam.mapService().save("map.mvs", [](auto res){ /* Result<void> */ });
slam.mapService().load("map.mvs", [](auto res){ /* Result<MapSummary> */ });
slam.mapService().localize(std::nullopt, [](auto res){ /* 다음 프레임들에서 로드 맵에 재측위, 성공 시 그 맵에서 추적 계속 */ });
auto where = slam.localize(frame);               // 무상태 단일 프레임 재측위 (Result<Pose>)
```

## 저장소 구조
`include/mvslam/api`(공개 API) · `src/{core,sensors,map,frontend,backend,pipelines,api}` · `apps/`(데이터셋 러너) · `tests/` · `configs/` · `third_party/`(서브모듈) · `cmake/` · `docker/` · `docs/`.

## 라이선스
미정 — `docs/adr/0008-license-clean-room-pending.md`.
