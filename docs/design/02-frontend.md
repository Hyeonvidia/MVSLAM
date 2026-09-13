# 02 — 프론트엔드 설계 (호출 스레드)

> 계보: OV²SLAM(`분석/02`)의 KLT 파이프라인을 골격으로, ORB-SLAM3(`분석/01` §2)의 상태기계·IMU 예측·재측위·초기화(H/F)·close-point KF 규칙을 결합한다. 상수 기본값의 출처는 `분석/04`.
> 원칙: **프레임당 비용 고정**(KLT 2패스 + PnP), **descriptor는 KF와 LOST에서만**, 모드 차이는 전략 객체 교체로만 표현.

## 1. 인터페이스 (`src/frontend/`)

| 인터페이스 | 책임 | 기본 구현 | 대안 |
|---|---|---|---|
| `ImagePreprocessor` | 그레이 변환, CLAHE(옵션), 광류 피라미드 구축, 이전 피라미드 swap | `OpenCvPreprocessor` | GPU 버전(후순위) |
| `IFeatureTracker` | 이전→현재 프레임 트랙 전파(2단), fb-check, 트랙 id 유지 | `KltTracker` | `DescriptorMatcherTracker`(ORB 매칭, 비교 실험용) |
| `IOutlierFilter` | 2D-2D 기하 필터 | `EssentialRansacFilter`(5-pt) | `FundamentalRansac` |
| `IPosePredictor` | 다음 프레임 pose 초기값 | `ConstantVelocityPredictor` | `ImuPredictor` |
| `IPoseOptimizer` *(frontend 선언, backend 구현)* | 프레임 pose 확정 | `MotionOnlyBa` | `PoseInertialOptimizer` |
| `IKeyframePolicy` | KF 판정 | `ParallaxTrackRatioPolicy` | + `ClosePointRule`, `InertialTimeRule` 데코레이터 |
| `IInitializer` | 맵 부트스트랩 | `TwoViewHFInitializer`(mono) / `DepthInitializer`(stereo·rgbd) | `FivePointInitializer` |
| `IRelocalizer` | LOST 시 재측위 | `BowPnpRelocalizer` | — |
| `TrackingStateMachine` | OK / RECENTLY_LOST / LOST 전이, 센서별 연명 | 단일 구현(State 패턴) | — |

프레임 데이터는 `frontend::Frame`(내부)에 담고, KF 승격 시 `map::KeyFrame`으로 **복사**(트랙 좌표·id·피라미드 참조·우/깊이 영상 참조·preintegration). 프론트엔드는 `map::Map`을 **읽기만** 하고(로컬맵 MP 위치·descriptor), 쓰기는 `KeyframeRequest` 큐로만 한다.

## 2. 단계별 설계

### 2.1 전처리
- CLAHE `clip=3.0, tile 8×8`(OV²SLAM accurate). 검출은 CLAHE 영상, **descriptor는 원본 영상**(`분석/02` §2.1 — 응답은 돕고 기술자 일관성은 해치지 않게).
- 피라미드: `cv::buildOpticalFlowPyramid(win=9, maxLevel=3)` → 4레벨(scale 2). 이전 프레임 피라미드는 `std::swap`으로 재사용(OV²SLAM).
- 스테레오/RGB-D의 우/깊이 영상은 **프레임 경로에서 처리하지 않는다**(KF 큐에 참조만 실어 Mapper가 처리) — 프레임당 단안 비용 유지(`분석/02` §0).

### 2.2 추적 (`KltTracker`)
| 단계 | 규칙 | 근거 |
|---|---|---|
| 3D 트랙 | 예측 pose로 MP를 투영한 픽셀을 초기값으로 **2레벨**(maxLevel=1) KLT | `분석/02` §2.2 |
| 2D 트랙 + 3D 실패분 | 이전 위치 초기값, **4레벨** | 동일 |
| 검증 | 순방향 err ≤ 30, 역방향(레벨 0만) 왕복 ≤ 0.5 px, 이미지 경계 | `분석/04` §9.4 |
| 붕괴 감지 | 3D 트랙 생존율 < 0.33 → `p3p_required` 플래그 | `분석/02` §2.3 |
| 트랙 id | KF에서 부여된 `TrackId` = `MapPointId` 후보; 프레임은 `unordered_map<TrackId, Keypoint>` 보유(OV²SLAM `Frame`) | `분석/06` §6.4 |

정보행렬 Σ: KLT에는 octave가 없으므로 기본 **Σ = I**(OV²SLAM 실측 0.032 m로 충분). 옵션으로 관측 품질 가중(`w = f(fb_err, track_age)`)을 `Observation.info`에 실어 백엔드가 그대로 사용한다(cuVSLAM `observation_infos` 슬롯, `분석/06` §6.4).

### 2.3 아웃라이어 필터 (`EssentialRansacFilter`)
- 회전 보상 median parallax > 6 px일 때만 실행(순수 회전에서 E 퇴화 회피), outlier > 50%면 퇴화로 간주 전체 스킵.
- stereo && 3D 트랙 > 30이면 3D 트랙으로 E를 추정하고 2D 트랙은 Sampson 거리로 필터(OV²SLAM 2단 트릭).
- 구현: OpenCV `findEssentialMat(RANSAC, 100 iter, 3 px→각오차 변환)`. OpenGV 비의존.

### 2.4 자세 추정
```
pose ← IPosePredictor
if (p3p_required || cfg.always_p3p)  pose ← P3P-RANSAC(OpenCV SQPnP/AP3P, 100 iter, 3 px)
pose ← IPoseOptimizer.optimize(pose, 3D 관측)   // MotionOnlyBa: Huber √5.9915, 최대 5 iter, 5 ms 상한, χ²/음수 depth 제거 후 L2 재정련
실패(NaN/inlier<15) → p3p_required=true → 다음 프레임 재시도; 연속 실패 → RECENTLY_LOST
```
관성 모드: `PoseInertialOptimizer`가 15-D 상태(`T_wb, v, b`)를 시각 관측 + IMU factor + **직전 프레임 prior**(15×15, 백엔드 `Marginalizer`가 생성)로 풀고, 맵 갱신 직후 프레임은 마지막 KF 고정(ORB CASE A/B, `분석/01` §2.4). IMU 초기화 완료 후에는 등속 모델을 쓰지 않는다.

### 2.5 상태기계 (`TrackingStateMachine`, State 패턴)
| 상태 | 전이 | 근거 |
|---|---|---|
| `NotInitialized` | `IInitializer` 성공 → `Ok` | |
| `Ok` | inlier < θ(관성 15 / 시각 30 / 재측위 직후 50) → `RecentlyLost` | `분석/01` §2.6 |
| `RecentlyLost` | 관성: IMU 추측항법으로 5 s 연명(재측위 시도 안 함); 시각: 매 프레임 재측위 시도, 3 s | ORB 비대칭 정책 |
| `Lost` | 현재 맵 KF > 10 → `MapStore.startNewMap()`(기존 맵 보존, 병합은 Loop 워커) / ≤ 10 → 맵 리셋 | Atlas 5요소 |
| 시간 점프 | 타임스탬프 역행 또는 > 1 s 점프 → 새 맵(관성: VI-BA2 미완이면 리셋) | |

재측위(`BowPnpRelocalizer`): **LOST 상태에서만 프레임에 ORB descriptor를 계산**(`분석/06` §6.3) → 모든 맵의 `KeyframeDatabase` 질의(ORB는 active 맵 한정 — 개선) → BoW 매칭 ≥ 15 → PnP-RANSAC(≥ 6점) → `MotionOnlyBa` → 투영 보강 2단 → inlier ≥ 50.

### 2.6 KF 판정 (`IKeyframePolicy` + 데코레이터)
| 규칙 | 값 | 출처 |
|---|---|---|
| 즉시 KF | 점유 셀 < 0.33·max && ≥ 5 프레임 && Optimizer 유휴; 또는 3D 트랙 < 20 && ≥ 2 프레임 | OV²SLAM |
| 억제(역압) | 3D 트랙 > 0.5·max && (Optimizer 진행 중 ‖ < 2 프레임) | OV²SLAM `blocalba_is_on_` |
| 일반 | (median parallax ≥ 20 px ‖ 3D < 0.75·lastKF ‖ (점유 < 0.5·max && 3D < 0.85·lastKF)) && parallax ≥ 10 px | `분석/02` §2.4 |
| stereo/rgbd 데코 | 마지막 KF 후 1 s 강제; close-point 규칙 `tracked_close < 100 && new_close > 70` | OV²SLAM / ORB τ_t·τ_c |
| 관성 데코 | 마지막 KF 후 0.5 s(초기화 후) / **0.25 s(초기화 전, 4 Hz)**; RECENTLY_LOST 중 `insert_kf_when_lost` 옵션 | ORB |
| 호출 정책 | Mapper 큐가 가득(> 2)이면 stereo류는 강행 삽입, mono는 포기(ORB) — 큐는 `LatestQueue`라 사실상 drain | `분석/03` §1 |

### 2.7 KF 시점 측정 추출 (Mapper 워커에서 실행되지만 프론트엔드 모듈이 제공)
- 검출 `GridShiTomasiDetector`: 셀 35 px(accurate)/45/50(프리셋), **기존 트랙 점유 셀 스킵**, 셀당 최강 1점 + `cornerSubPix`, 적응 품질(검출률 < 33% → 임계 ½, > 90% → ×1.5). 대안 `GridFastDetector`.
- 기술 `OrbDescriptorExtractor`(단일 레벨, 회전 보정) — 재측위·루프·로컬맵 재관측용. OpenCV `ORB::create(…, nlevels=1)`. BRIEF(xfeatures2d)는 의존성 때문에 비채택(OV²SLAM 실측: ORB 폴백과 동등).
- 깊이 `IDepthSource`: `KltStereoMatcher`(3D 트랙은 투영 prior 1레벨, 2D 트랙은 이웃 3D ≥ 3개 깊이 보간 prior 후 4레벨; rectified면 |Δy|, 아니면 Sampson ≤ 2 px), `DepthImageSource`(유효 범위 [min,max], σ_z = z²/(f·b_virtual), b_virtual 기본 0.08 m).
- close/far: `depth < ThDepth·baseline`(EuRoC 60·b) — KF 시 즉시 MP 생성 대상(ORB), far는 temporal 삼각측량 대기.

### 2.8 초기화
| 모드 | 방법 | 비고 |
|---|---|---|
| Mono | 두 프레임 트랙 ≥ 100, 회전 보상 parallax ≥ 20 px → **H/F 병렬 8-pt RANSAC(200회)**, `R_H = S_H/(S_H+S_F) > 0.45`(논문값 채택; ORB 코드는 0.50 — `분석/00` DIFF) → 삼각측량 → GBA 20 iter → median depth 정규화(순수 1.0 / 관성 4.0) | ORB `TwoViewReconstruction` 의미론 |
| Stereo / RgbD | 트랙 ≥ 500(격자 검출) → 첫 프레임 KF + depth > 0 전부 MP | ORB |
| 관성(공통) | 시각 초기화 후 `ImuInitializer` 상태기계(백엔드)가 scale/gravity/bias 사후 추정; 초기화 전 KF 4 Hz 강제, 가속도 변화 ≥ 0.5 m/s² 요구(stereo-I) | [03-backend.md](03-backend.md) §5 |

### 2.9 IMU 예측 (`ImuPredictor`)
- 두 누산기 유지: `preint_from_last_kf`(최적화용)·`preint_from_last_frame`(예측용). 경계 샘플 선형보간.
- 앵커 규칙: 맵 갱신 epoch가 바뀐 프레임은 **마지막 KF 앵커**, 아니면 **직전 프레임 앵커**(ORB `PredictStateIMU`). bias는 앵커의 bias, Δ는 1차 Jacobian 보정.
- 프레임 pose 출력은 항상 `T_wb`; 카메라 pose는 rig로 파생.

## 3. 프레임 출력과 궤적
- `Frame`은 참조 KF 기준 상대 pose `T_kb`(ORB `mTcp`, OV²SLAM logger)를 함께 저장한다. PGO/GBA로 KF가 보정되면 프레임 궤적은 `T_wk·T_kb`로 자동 갱신된다 — `System::trajectory()`가 이를 재구성한다.
- `TrackResult{timestamp, optional<PoseState>, TrackingState, stats{n_tracks, n_3d, n_inliers, kf_inserted}}`.

## 4. 파라미터 (`configs/*.yaml` `frontend:` 절)
| 키 | 기본 | 출처 |
|---|---|---|
| `preprocess.clahe {enable, clip}` | true, 3.0 | OV²SLAM accurate |
| `tracker.win_size / pyr_levels / fb_max_dist_px / max_err` | 9 / 3 / 0.5 / 30 | OV²SLAM |
| `tracker.p3p_survival_ratio` | 0.33 | OV²SLAM 코드(논문 0.5) |
| `outlier.parallax_gate_px / ransac_iters / err_px` | 6 / 100 / 3 | OV²SLAM |
| `pose.huber_chi2 / max_iters / max_time_ms` | 5.9915 / 5 / 5 | OV²SLAM |
| `keyframe.min_parallax_px / track_ratio / occupancy_ratio / stereo_force_s / imu_interval_s` | 20 / 0.75 / 0.5 / 1.0 / 0.5 | OV²SLAM·ORB |
| `keyframe.close_point {tracked_min, new_min, depth_factor}` | 100 / 70 / 60 | ORB(EuRoC yaml) |
| `detector.cell_px / type / subpix` | 35 / shi_tomasi / true | OV²SLAM |
| `depth.stereo_epipolar_px / rgbd_virtual_baseline_m / min_depth / max_depth` | 2 / 0.08 / 0.1 / 40 | OV²SLAM·ORB2 |
| `state.recently_lost_s {inertial, visual} / new_map_min_kf` | 5 / 3 / 10 | ORB |
| `reloc.min_bow_matches / min_inliers` | 15 / 50 | ORB |

## 5. 테스트 계획
- `KltTracker`: 합성 이동 영상(체커보드 워핑)으로 트랙 정확도·fb-check 기각률 검증.
- `EssentialRansacFilter`: 합성 대응점 + 주입 outlier 30% → 정확 E 복원.
- `IKeyframePolicy`: 상태 시나리오 테이블 기반 결정 테스트(역압·시간 조건).
- `TrackingStateMachine`: 전이표 전수 테스트(관성/시각 비대칭 포함).
- 통합: EuRoC MH01 stereo(Offline 모드) ATE 게이트([09-roadmap.md](09-roadmap.md)).
