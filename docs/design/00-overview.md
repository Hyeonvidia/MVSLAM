# 00 — 개요: 목표, 원칙, 설계 입력

> MVSLAM(**M**odular **V**isual **SLAM**, 잠정명 — 네임스페이스 `mvslam`, 변경 시 `sed` 1회)은
> mono / stereo / RGB-D × (±IMU) 모드를 하나의 코드베이스로 지원하는 **멀티플랫폼 Modern C++20 Visual SLAM 라이브러리**다.
> 방법론은 OV²SLAM(프론트엔드·실시간 장치)과 ORB-SLAM3(백엔드·맵 계층·VI 정식화)에서, 아키텍처는 cuVSLAM(명시적 API·비동기 워커·결정적 오프라인 모드·의존성 고정)에서 가져온다.
> 근거는 전부 `/Users/jhpark/VSLAM/code-analysis/analysis/`(이하 `분석/`)의 코드 실측(`[CODE]`)이다. 결정 ↔ 근거 대응표는 [10-traceability.md](10-traceability.md).

## 1. 목표 / 비목표

| 구분 | 내용 |
|---|---|
| **지원 모드** | `Mono`, `Stereo`, `RgbD` × `{Visual, VisualInertial}` = 6 모드(ORB-SLAM3 `eSensor` 6종과 동일 범위, `분석/01` §1) + **N-카메라 rig 정식화**(ORB `ImuCamPose`·cuVSLAM rig, `분석/01` §6) |
| **기능** | 실시간 추적, 로컬 BA, 루프 클로징(PGO + loose BA), relocalization, 다중 맵(LOST → 새 맵 + 병합), 맵 저장/재사용(IMU 상태 포함), 이벤트 훅(dense mapping 연동용) |
| **플랫폼** | macOS(Apple Silicon, 개발), Linux x86_64(CI), Linux aarch64 = Jetson Orin(Docker), iOS(정적 XCFramework, 후순위). Windows는 비목표(코드는 배제하지 않음) |
| **품질 목표** | 유지보수성(모듈 경계·디자인 패턴·문서화), 결정성(오프라인 모드 재현), 호스트를 죽이지 않음(라이브러리), 메모리 상한 |
| **비목표(현 단계)** | dense mapping 자체(훅만 제공, `분석/07` §7.1), GNSS/다중 센서 융합(VIGIL 영역), 학습 기반 특징, ROS1, GPU 커널(설계는 교체 가능하게 두되 P0~P3는 CPU) |

## 2. 설계 입력 (무엇에서 무엇을 가져오는가)

| 계보 | 가져오는 것 | 근거 |
|---|---|---|
| **OV²SLAM** | KLT 프론트엔드(2단 피라미드·fb-check), descriptor는 KF에서만, 격자 검출, KF 시점 좌→우 KLT 스테레오, anchored inverse-depth BA 파라미터화, 해석 Jacobian cost function, robust→L2 2-pass, 시간 상한·선점·drain·역압, SE3 PGO + loose BA, 5단 검증 게이트, 모드 = yaml 프리셋 | `분석/02`, `분석/03` §2, `분석/04` §9.4 |
| **ORB-SLAM3** | body-frame 상태(`T_wb`, N-카메라 파생), IMU preintegration 정식화(공분산·bias Jacobian), inertial-only MAP 초기화 + staged prior, pose-inertial 추적(직전 프레임 prior = Schur marginalization), 시간축 관성 LBA(N=10) + 경계 다운웨이트, covisibility/essential 이중 그래프, place recognition 다단 검증(공간 3표+시간 3표), Sim3/SE3/4-DoF 보정, Atlas 5요소, 직렬화(안정 id·포인터→id) | `분석/01`, `분석/05`, `분석/10` §2 |
| **DM-VIO** | marginalization 모듈(Schur + FEJ, 라이브러리 독립), delayed marginalization 개념(재초기화 가능 추정기), 초기화 상태기계 + scale 분산 단일 판정 지표, 동적 광도 가중 개념(=시각/관성 가중 조절) | `분석/08`, `분석/10` §4·§6 |
| **cuVSLAM** | 공개 API 3클래스 + Config/Internals 분리·setter 금지·모순 설정 throw, 호출 스레드 + 워커(SBA·SLAM) 비동기, 생산자 비블록·retention window, Offline = 같은 워커 코드 동기 실행(결정성), 고정 윈도우 SBA(7 KF), 의존성 전부 고정(SHA/태그)·단일 라이브러리·curated 패키지, KF 선택(리그 전역)과 특징 보충(카메라별) 분리, 평가 게이트 | `분석/09`, `cuVSLAM/cuvslam_design_analysis.md` §4·§7·§11 |
| **ORB_SLAM3_Modern 교훈** | `EdgeInertialGS` scale Jacobian(가산) vs 갱신(승산) 불일치 결함 → 우리 정식화에서 제거; 소유권 계약(shared_ptr + tombstone/epoch); g2o 최신 태그·Eigen 5 조합 검증 완료 | 메모리 `project-orb-slam3-modern` |
| **VINS_Remastered 교훈** | iOS 크로스컴파일 시 Ceres `MINIGLOG`+`EIGENSPARSE`, OpenCV `BUILD_LIST` 트리밍, 코어/어댑터(Hexagonal) 분리 | `VINS_Remastered/cmake/Dependencies.cmake` |
| **VIGIL 원칙** | 라이선스가 설계를 규정(permissive 라이브러리 + clean-room), C ABI 경계(후순위 래퍼), 결정성 1급, 호스트를 죽이지 않음, 메모리 상한, 관측 비용 0 | `VIGIL/docs/design/architecture.md` §2–§4 |

## 3. 10대 설계 원칙

1. **프론트엔드는 descriptor-free, 측정 추출은 KF에서만.** 프레임당 비용을 KLT 2패스 + PnP로 고정한다(OV²SLAM·cuVSLAM 계보; `분석/02` §2.1, `분석/06` §6.3). ORB descriptor는 KF 생성 시와 LOST 상태에서만 계산한다.
2. **백엔드 상태는 body frame 하나.** 모든 최적화 정점은 `T_wb`(+v, b), 카메라는 rig extrinsic으로 파생한다(ORB `ImuCamPose`의 N-camera 구조를 시각 전용 모드까지 확장, `분석/05` §3-1의 "pose 규약 3종" 문제를 원천 제거).
3. **Factor가 수식을 소유하고, 솔버는 어댑터다.** 잔차·해석 Jacobian은 `mvslam::graph` 타입에 있고 Ceres/g2o/GTSAM은 이를 감싸는 얇은 어댑터다. 이것이 (a) 솔버 교체, (b) 라이브러리 독립 marginalization, (c) 단위 테스트(수치 미분 대조)를 가능하게 한다(`분석/10` §1 — 세 라이브러리 모두 marginalization API가 없다는 사실이 근거).
4. **점은 XYZ가 정본, BA 안에서만 anchored inverse-depth.** 저장·루프 보정·직렬화는 XYZ, 최적화 파라미터화는 anchor KF 기준 ρ(1-DoF)로 문제 크기를 줄인다(OV²SLAM 방식, `분석/06` §6.2 — Sim3 보정 시 ρ/s).
5. **과거 상태 = fix + culling(SLAM) / prior(추적).** 맵 최적화는 고정 KF + KF culling(ORB·OV²SLAM), 프레임 추적은 직전 프레임을 Schur marginalize한 15-D prior(ORB)로. 슬라이딩 윈도우 marginalization(dm-vio식)은 같은 `Marginalizer` 모듈을 쓰는 **VIO 전용 모드의 옵션**이다.
6. **모드는 코드 분기가 아니라 합성이다.** `PipelineFactory`가 `SensorMode × InertialMode`에 따라 전략 객체(추적기·깊이 소스·pose 예측기·윈도우 선택기·보정기)를 조립한다. `if(mSensor==...)` 산재(ORB `Tracking.cc`)를 금지한다.
7. **실시간 장치는 계층마다 명시.** 프레임 drain, KF drain(+covisibility 합류), BA 선점·시간 상한, 백엔드 부하 → KF 억제 역압, retention window(늦은 보정 폐기). "늦으면 버리되 정보는 회수"(OV²SLAM·cuVSLAM, `분석/03` §5).
8. **결정성은 1급.** `ExecutionMode::Offline`은 워커 코드를 호출 스레드에서 동기 실행하고 시드를 고정한다. 평가는 전부 Offline으로 한다(cuVSLAM). 폴링 bool 플래그 대신 큐 + condition_variable + `std::atomic`.
9. **공개 API는 얇고 불변.** cuVSLAM식 `Config` 1회 검증·setter 없음·모순 설정은 생성자에서 throw·튜닝 노브는 `internals` 헤더로 격리. 공개 헤더에 Eigen/OpenCV/Ceres 타입을 노출하지 않는다(pimpl + POD). C ABI(`mvslam_c.h`)는 이 위의 래퍼로 후순위 추가(VIGIL).
10. **의존성은 전부 서브모듈 태그 고정, 빌드는 CMake 프리셋.** Eigen·Sophus·Ceres·g2o·GTSAM·OpenCV·yaml-cpp·spdlog·gtest·cereal·benchmark를 shallow 서브모듈로 두고 태그를 [07-build-platforms.md](07-build-platforms.md) 표에 고정한다. `-march=native` 금지.

## 4. 관련 프로젝트와의 관계

| 프로젝트 | 관계 |
|---|---|
| `VIGIL` | 상위 목표(VI-GNSS 다중센서 L&M)의 설계 지도. MVSLAM은 VIGIL이 채택할 수 있는 **시각(+관성) SLAM 코어 후보**이며 VIGIL 원칙(§2 표)을 따른다. GNSS·다중센서 융합은 MVSLAM 밖이다. |
| `ORB_SLAM3_Modern` / `ORB_SLAM3_Remastered` | 업스트림 ORB-SLAM3의 리팩토링(GPL 파생). MVSLAM은 **코드를 가져오지 않고** 수식·의미론과 그 프로젝트의 교훈(결함·소유권·빌드 조합)을 가져온다. |
| `VINS_Remastered` | 플랫폼 독립 VIO 코어 프로젝트. 빌드/플랫폼 패턴(iOS, Docker, FetchContent 설정값)을 재사용한다. 의존성 관리 방식은 다르다(그쪽 FetchContent, 이쪽 서브모듈 — 사용자 지시). |
| `cuVSLAM`(NVIDIA) | 아키텍처 참조. 라이선스가 NVIDIA 플랫폼 한정이라 **코드 링크·이식 불가**(iOS·macOS 타깃), 설계 개념만 사용. |

## 5. 라이선스 — 결정 항목 (미확정)

- 방법론 원천 ORB-SLAM3·OV²SLAM·DM-VIO는 **GPL-3.0**이다. MVSLAM이 permissive(BSD-3/Apache-2)를 유지하려면 **clean-room**(논문 + `분석/` 문서 수준의 서술로부터 재구현, 코드 복사 금지)이어야 한다. 반대로 GPL 코드를 한 줄이라도 이식하면 MVSLAM 전체가 GPL-3.0이 된다.
- 선택된 서드파티는 전부 permissive: Eigen(MPL2), Sophus(MIT), Ceres/g2o/GTSAM(BSD), OpenCV(Apache-2), yaml-cpp/spdlog(MIT), gtest/cereal/benchmark(BSD/Apache).
- 결정 전까지 `LICENSE` 파일을 두지 않는다. **기본 가정: clean-room, permissive 지향.** 설계는 어느 쪽이든 유효하다.

## 6. 용어

| 용어 | 정의 |
|---|---|
| body frame `b` | 최적화·출력의 기준 좌표계. IMU가 있으면 IMU, 없으면 rig 기준 카메라(camera 0)와 일치시킨다. `T_wb` = body→world |
| rig | 카메라 N대 + IMU의 extrinsic 집합(`T_bc_i`). 시각 전용 mono면 N=1, `T_bc0 = I` |
| frame / keyframe(KF) | 매 입력 시각의 추적 단위 / 맵에 보존되는 프레임(측정 추출·삼각측량·BA 대상) |
| map point(MP) | 3D 랜드마크. 정본은 XYZ, BA 파라미터화는 anchored inverse depth |
| anchor | MP의 역깊이가 정의되는 KF(첫 관측 KF, 삭제 시 다음 관측 KF로 이전) |
| covisibility graph / essential graph | KF 간 공유 MP ≥ θ_cov 간선 그래프 / spanning tree + loop edge + 고가중 간선의 희소 부분 그래프(PGO용) |
| local BA(LBA) | 윈도우(공시성 또는 시간축) 내 KF·MP를 푸는 BA. 윈도우 밖 관측 KF는 고정 |
| prior / marginalization | 제거되는 변수의 정보를 Schur complement로 남은 변수의 2차 항으로 접는 것. 결과는 `PriorFactor` |
| PGO | pose graph optimization — KF pose만, 상대 pose 제약 |
| Realtime / Offline | 실행 모드. Realtime은 워커 비동기·프레임 drain, Offline은 동기·결정적 |
