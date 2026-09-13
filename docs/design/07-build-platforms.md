# 07 — 빌드, 의존성(서브모듈), 플랫폼

> 사용자 지시: 백엔드 라이브러리(GTSAM/Ceres/g2o 등)는 **git 서브모듈**로 관리하고 **최신 릴리스**로 고정한다. cuVSLAM의 "의존성 전부 고정 + 단일 라이브러리 산출" 원칙, VINS_Remastered의 iOS 설정값을 재사용한다. `-march=native`는 금지(ORB_SLAM3·OV²SLAM 이식 함정).

## 1. 서브모듈 핀 (`third_party/`, 전부 shallow)
| 이름 | 저장소 | 태그 | 라이선스 | 용도 | 필수 |
|---|---|---|---|---|---|
| eigen | gitlab.com/libeigen/eigen | **5.0.1** | MPL2 | 선형대수(단일 Eigen — 코어·Ceres·g2o·GTSAM 전부 이 하나) | ✔ |
| sophus | github.com/strasdat/Sophus | **1.24.6** | MIT | SE3/Sim3/SO3(헤더 전용) | ✔ |
| ceres-solver | github.com/ceres-solver/ceres-solver | **2.2.0** | BSD-3 | 1차 솔버 어댑터 | ✔(`MVSLAM_WITH_CERES`) |
| g2o | github.com/RainerKuemmerle/g2o | **20241228_git** | BSD-2/3 | 옵션 어댑터 | `MVSLAM_WITH_G2O` |
| gtsam | github.com/borglab/gtsam | **4.3a2** | BSD-3 | 옵션 어댑터(4.3 계열은 boost 제거) | `MVSLAM_WITH_GTSAM` |
| opencv | github.com/opencv/opencv | **4.14.0** | Apache-2 | KLT·검출·PnP·이미지 I/O. 시스템 OpenCV 우선, 없거나 iOS면 서브모듈 트리밍 빌드 | ✔(`MVSLAM_OPENCV_SOURCE`) |
| yaml-cpp | github.com/jbeder/yaml-cpp | **yaml-cpp-0.9.0** | MIT | 설정 파서 | ✔ |
| spdlog | github.com/gabime/spdlog | **v1.17.0** | MIT | 로깅(fmt 내장) | ✔ |
| googletest | github.com/google/googletest | **v1.18.0** | BSD-3 | 단위 테스트 | `MVSLAM_BUILD_TESTS` |
| cereal | github.com/USCiLab/cereal | **v1.3.2** | BSD-3 | 맵 직렬화(헤더 전용) | ✔ |
| benchmark | github.com/google/benchmark | **v1.9.5** | Apache-2 | 마이크로벤치 | `MVSLAM_BUILD_BENCH` |
- 정확한 커밋은 `.gitmodules` + `git submodule status`가 정본이다. 갱신은 `scripts/bump_submodule.sh <name> <tag>`로만 하고 ADR에 사유를 남긴다.
- 최초 체크아웃: `git clone --recurse-submodules --shallow-submodules` 또는 `scripts/bootstrap.sh`.
- 알려진 조합: Eigen 5.0.1 + g2o 20241228_git(ORB_SLAM3_Modern R2에서 검증), Eigen 5.0.1 + Ceres 2.2.0(Homebrew 동일 조합). GTSAM 4.3a2 + Eigen 5는 **미검증**(`GTSAM_USE_SYSTEM_EIGEN=ON` 필수, 옵션 OFF 기본).
- DBoW3/fbow 등 place recognition 라이브러리는 P3 착수 시 결정(`IPlaceRecognizer` 뒤에 숨김).

## 2. CMake 구조
```
CMakeLists.txt              프로젝트·옵션·서브디렉터리
cmake/Options.cmake         MVSLAM_* 옵션 정의·검증(모순 시 FATAL_ERROR)
cmake/Dependencies.cmake    서브모듈 → 타깃 배선 (Eigen INTERFACE, Sophus INTERFACE, Ceres/g2o/GTSAM add_subdirectory + 옵션 고정)
cmake/OpenCV.cmake          system | submodule | none 선택, 트리밍 BUILD_LIST, include 경로 함수
cmake/Warnings.cmake        mvslam_set_warnings(target) — -Wall -Wextra -Wpedantic (+ -Werror in CI)
cmake/Sanitizers.cmake      MVSLAM_SANITIZER=none|asan|tsan|ubsan
cmake/toolchains/ios.cmake  iOS 툴체인 변수
src/*/CMakeLists.txt        모듈별 OBJECT/STATIC 타깃 mvslam_core, mvslam_sensors, mvslam_map, mvslam_frontend, mvslam_backend, mvslam_pipelines, mvslam_api
```
| 옵션 | 기본 | 설명 |
|---|---|---|
| `MVSLAM_WITH_CERES` | ON | 1차 솔버 |
| `MVSLAM_WITH_G2O` / `MVSLAM_WITH_GTSAM` | OFF | 옵션 어댑터(프리셋 `*-full`에서 ON) |
| `MVSLAM_OPENCV_SOURCE` | `auto` | `auto`(find_package 실패 시 submodule) / `system` / `submodule` / `none`(코어·백엔드만 빌드 — CI 단위 테스트용) |
| `MVSLAM_BUILD_TESTS` / `MVSLAM_BUILD_APPS` / `MVSLAM_BUILD_BENCH` | ON / ON / OFF | iOS 프리셋은 전부 OFF |
| `MVSLAM_SINGLE_LIBRARY` | ON | 모듈 OBJECT 라이브러리를 `libmvslam`(shared 또는 static) 하나로 병합(cuVSLAM) |
| `MVSLAM_SANITIZER` | none | |
| `MVSLAM_WERROR` | OFF(CI ON) | |
- 서드파티 옵션 고정(Dependencies.cmake): Ceres `MINIGLOG=ON, EIGENSPARSE=ON, SUITESPARSE=OFF, CXSPARSE=OFF, BUILD_TESTING=OFF, BUILD_EXAMPLES=OFF, BUILD_BENCHMARKS=OFF, PROVIDE_UNINSTALL_TARGET=OFF, MINIGLOG_MAX_LOG_LEVEL=0`; g2o `G2O_BUILD_APPS=OFF, G2O_BUILD_EXAMPLES=OFF, G2O_USE_OPENGL=OFF, G2O_BUILD_SLAM2D_TYPES=OFF ...`; GTSAM `GTSAM_USE_SYSTEM_EIGEN=ON, GTSAM_WITH_TBB=OFF, GTSAM_BUILD_TESTS/EXAMPLES/UNSTABLE=OFF`; OpenCV(submodule) `BUILD_LIST=core,imgproc,features2d,calib3d,video(,imgcodecs)`, `WITH_EIGEN=OFF`(Eigen 결합 회피), GUI/ffmpeg/IPP/ITT OFF.
- Eigen은 `add_subdirectory` 하지 않고 `Eigen3::Eigen` INTERFACE 타깃 + `CMAKE_FIND_PACKAGE_REDIRECTS_DIR`에 config를 써서 Ceres/g2o/GTSAM의 `find_package(Eigen3)`를 서브모듈로 리다이렉트한다(VINS_Remastered·ORB_SLAM3_Modern 방식).
- 컴파일 표준: 우리 타깃 C++20(`CXX_STANDARD 20, CXX_EXTENSIONS OFF`); 서드파티는 자기 기본값 유지(g2o에 strict c++17 강제 시 성능 저하 사례 — ORB_SLAM3_Modern).
- 최적화 플래그: Release `-O2 -DNDEBUG`, `-fvisibility=hidden` + `MVSLAM_EXPORT` 매크로, LTO 옵션(`MVSLAM_LTO`). **`-march=native` 금지**, aarch64는 기본 armv8-a(+NEON), x86_64는 기본(SSE2)+옵션 `-mavx2` 프리셋.

## 3. 프리셋 (`CMakePresets.json`)
| 프리셋 | 대상 | 특징 |
|---|---|---|
| `macos-release` / `macos-debug` | Apple Silicon 개발 | Ninja, 시스템 OpenCV 없으면 submodule |
| `macos-full` | 〃 | g2o + GTSAM ON |
| `linux-release` / `linux-asan` / `linux-tsan` | x86_64·aarch64 CI/Docker | |
| `jetson-release` | Jetson Orin(JetPack 6, Ubuntu 22.04, gcc 11) | aarch64, CUDA 미사용(P0~P3) |
| `ios-device` / `ios-simulator` | iOS 17+ | `CMAKE_SYSTEM_NAME=iOS`, 정적, 테스트/앱 OFF, OpenCV submodule(imgcodecs 제외), Ceres MINIGLOG |
빌드 예(도커): `scripts/docker.sh image && scripts/docker.sh build` (= `docker compose run --rm dev docker/scripts/container_build.sh docker-release`). macOS/iOS 프리셋은 플랫폼 이식 검증용이며 일상 빌드는 도커다.

## 4. 플랫폼별 주의
| 플랫폼 | 툴체인 | 주의 |
|---|---|---|
| macOS 15 (arm64) | Apple clang 17+ | Homebrew OpenCV 있으면 system, 없으면 submodule 빌드(약 10분) |
| Ubuntu 24.04 x86_64 | gcc 13 / clang 17 | CI 기본 |
| Jetson Orin (JetPack 6, Ubuntu 22.04) | gcc 11 | C++20 부분 지원 범위 안에서만 사용(`std::jthread`·`std::stop_token`·concepts OK; `std::format` 미사용 → spdlog/fmt) |
| iOS 17 | Xcode 16 | 정적 XCFramework, `platform/ios`에서 `xcodebuild -create-xcframework`; 파일 I/O 앱 없음 |
| Windows | MSVC 2022 | 비목표. 코드는 POSIX 전용 API를 쓰지 않는다(`std::filesystem`, `std::thread`) |

## 5. Docker — 유일한 빌드 경로 (사용자 지시 2026-09-13: 로컬 빌드 금지)
- 이미지 `docker/Dockerfile`(ubuntu:24.04, gcc-13/clang, cmake 3.28, ninja, ccache, 시스템 OpenCV 4.6). 라이브러리는 전부 서브모듈이므로 컨테이너 안팎의 빌드 경로가 같다.
- **Rerun 뷰어**([ADR-0011](../adr/0011-rerun-viewer.md)): 이미지 빌드 시 Rerun C++ SDK 0.37.2 릴리스 zip(sha256 고정)을 받아 `/opt/rerun`에 설치한다(SDK CMake가 Apache Arrow 18.0.0을 내려받아 정적 빌드 — 이미지 빌드 1회, 프로젝트 configure는 네트워크 불필요). `docker-*` 프리셋은 `MVSLAM_WITH_RERUN=ON` + `CMAKE_PREFIX_PATH=/opt/rerun`. 어댑터 `src/platform/viz/rerun_sink`(`mvslam::viz_rerun`)는 공개 API만 사용하며 libmvslam에 포함되지 않는다. 사용: 러너 `--rerun /results/run.rrd`(파일) 또는 `--rerun rerun+http://host.docker.internal:9876/proxy`(호스트에서 `rerun` 뷰어 실행 중일 때 실시간; compose가 `host.docker.internal`을 매핑). 호스트 뷰어는 `pip install rerun-sdk==0.37.2`(`.rrd`는 같은 버전의 뷰어로 연다). 렌더 내용: `world/cam`(+이미지·KLT 트랙/꼬리 매 N프레임), `world/kf/<id>` 프러스텀, `world/points`(KF N개마다 스냅샷), `world/trajectory`, `stats/**` 시계열(프론트엔드 ms, 3-D 트랙, mapper/LBA ms), `events` 텍스트 로그. 세계 좌표의 위쪽 축은 rig에서 유도(카메라0의 -y를 body로 회전한 지배 축 — EuRoC body는 +x가 위). 레이아웃 `viz/mvslam.rbl`. **브라우저 경로**: 이미지에 rerun-cli(Linux, sha256 고정)도 넣어 `scripts/docker.sh live`가 컨테이너 안에서 `rerun --serve-web`(웹 뷰어 :9090, gRPC :9876, compose `ports` + `--service-ports`)을 띄우고 러너를 `--execution realtime --pace 1`로 스트리밍한다 — 호스트에 아무것도 설치하지 않는다.
- `docker-compose.yml`: 서비스 `dev`(bind `.:/workspace`, `../Datasets:/datasets:ro`, `./results:/results`, 명명 볼륨 `mvslam-build:/workspace/build`·`mvslam-ccache:/ccache`), `dev-tsan`(seccomp unconfined — TSAN이 personality(2)로 ASLR을 끄기 때문).
- 프리셋 `docker-release` / `docker-full` / `docker-asan` / `docker-tsan`(linux-* 상속 + ccache 런처 + 시스템 OpenCV). 빌드 트리는 볼륨 안(`/workspace/build/<preset>`)에만 생긴다 — 호스트 `build/`는 비어 있어야 정상.
- 호스트 래퍼 `scripts/docker.sh {image|build|test|tsan|run|shell|clean}` → 컨테이너 스크립트 `docker/scripts/container_build.sh`(configure+build+ctest), `docker/scripts/run_dataset.sh <config> <EuRoC/MH01> [preset] [out] [--execution offline]`(결과는 `results/`; 평가는 `--execution offline`).
- Jetson Orin: 같은 Dockerfile을 JetPack 베이스(`nvcr.io/nvidia/l4t-base`)로 바꾼 `docker/Dockerfile.jetson`(P4)로 확장. CUDA는 P0~P3 미사용.
- CI(`.github/workflows/ci.yml`): 매트릭스 {ubuntu-24.04 gcc/clang, ubuntu-24.04-arm, macos-15} — CI 러너는 호스트 빌드지만 개발자 워크플로우는 도커다. `linux-tsan` nightly, `*-full` 주간.
- 캐시: 서브모듈 shallow + ccache 볼륨.

## 6. 코드 규약
- `.clang-format`(Google 기반, 100열, 포인터 좌측), `.clang-tidy`(modernize-*, readability-*, bugprone-*, performance-*, concurrency-*; cppcoreguidelines 일부).
- 헤더: `#pragma once`, 모듈 상단 3줄 주석(역할/스레드/불변식), 공개 헤더는 Doxygen.
- 네이밍: [01-architecture.md](01-architecture.md) §4.
- 커밋: Conventional Commits(`feat(frontend): …`), 서드파티 갱신은 `build(deps): bump g2o to …` + ADR.
