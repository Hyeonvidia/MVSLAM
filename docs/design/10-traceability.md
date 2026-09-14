# 10 — 결정 ↔ 근거 대응표

> 설계 결정마다 `분석/`(`/Users/jhpark/VSLAM/code-analysis/analysis/`)의 실측 항목이나 논문·선행 프로젝트 근거를 잇는다. 근거가 `[INFER]`인 결정은 그렇게 표시한다.

| 결정 | 근거 |
|---|---|
| KLT 프론트엔드 + KF/LOST에서만 descriptor | `분석/02` §2.1(describe 호출부 전수), `분석/06` §6.3(프레임 descriptor 요구 경로 목록), VIGIL 실측 MH01 OV²SLAM 0.032 m |
| 3D 트랙 2레벨 / 2D 4레벨 / fb-check 0.5 px | `분석/02` §2.2, `분석/04` §9.4 |
| P3P fallback 33% | `분석/00` "[DIFF] P3P fallback 조건은 33%"(코드가 논문 50%를 이김) |
| KF 규칙(20 px / 0.75 / 점유 0.5 / 즉시·억제 / stereo 1 s / IMU 0.5·0.25 s / close-point 100·70) | `분석/02` §2.4, `분석/01` §2.5, `분석/04` §9.1·9.3 |
| 상실 정책 비대칭(IMU 5 s / 시각 3 s), KF>10 새 맵 | `분석/01` §2.6, `분석/00` "[DIFF] 상실 판정 임계" |
| H/F 병렬 초기화, R_H>0.45(논문값) | `분석/04` §9.2(코드 0.50, 주석 "0.40-0.45") — 논문값 채택은 설계 판단 |
| body-frame 단일 pose 상태 + rig | `분석/01` §6(`ImuCamPose` N-camera), `분석/05` §3-1(규약 3종 문제), `분석/09` §1(cuVSLAM rig) |
| factor가 Jacobian 소유, 솔버 어댑터 | `분석/10` §1·§8(세 라이브러리 marginalization API 부재), `분석/02` §4(SizedCostFunction 해석 Jacobian), `분석/08` §3(DM-VIO H/b 자체 조립) |
| Ceres 1차, g2o/GTSAM 옵션 어댑터 | `분석/10` §8 권고, 사용자 지시(세 라이브러리 서브모듈), ORB_SLAM3_Modern P12-G0-2 등가성 봉투 방법론 |
| LogScale 파라미터화 | ORB_SLAM3_Modern P6-3 실측: `EdgeInertialGS` 가산 Jacobian vs 승산 갱신 → s*≥2 발산 |
| XYZ 정본 + anchored inverse depth(BA만), ρ/s | `분석/06` §6.2(SetWorldPos 20곳, OV²SLAM anchor 이전), `분석/02` §3.3 |
| 과거 상태 = fix + culling(SLAM), 프레임 prior = Schur marginalization | `분석/01` §3.3(`Marginalize` SVD Schur, `EdgePriorPoseImu`), `분석/10` §6 표 |
| Marginalizer 설계(Markov blanket, pseudo-inverse, PriorFactor) | `분석/08` §1(DM-VIO `marginalizeOut`), `분석/10` §4-6 |
| 시각 LBA 상한(20 KF) | `분석/00` "[CODE] 시각 LocalBundleAdjustment는 문제 크기 상한이 없다" |
| 관성 LBA 시간축 10/25 + 고정 ≤200, 경계 다운웨이트 | `분석/01` §3.2, `분석/00` "[DIFF] LocalInertialBA 윈도우" |
| LBA 선점·시간 상한 0.2 s·2-pass | `분석/02` §3.2, `분석/03` §2 |
| IMU preintegration 정식화(double), 재적분 임계 | `분석/01` §2.8, `분석/10` §7(cuVSLAM 재적분), `분석/00` "[CODE] preintegration bias 갱신은 1차 근사" |
| IMU 초기화: staged prior + scale 분산 판정 + 상태기계, stereo 1 s | `분석/01` §3.4(1e2/1e10·1e5→1/1e5→0/0, 1 s/2 s), `분석/08` §1(dm-vio 단일 지표·상태기계), `분석/00` "[DIFF] scale refinement 실효 3회" |
| 루프 검증 5단 ∪ 공간 3표 + 관성 게이트 | `분석/02` §3.4, `분석/01` §4, `분석/00` "[DIFF] place recognition 공간 검증은 3표" |
| 보정 Sim3/SE3/4DoF, PGO essential graph, loose BA, GBA 지연 적용 | `분석/01` §4, `분석/02` §3.4, `분석/00` "[MISSING] OptimizeEssentialGraph4DoF" |
| 병합 이원화(rigid snap vs welding), 현재 맵이 죽고 old 맵 생존 | `분석/01` §4(MergeLocal/MergeLocal2) |
| P3 구현: HBST 잎 탐색에서 제외 KF 건너뛰기 | 루프 질의는 질의 KF 자신이 색인에 있어 최근접 항목이 항상 제외 대상 — 표를 버리지 않고 다음 항목을 본다(설계 03 §5.1, 단위 테스트) |
| P3 구현: PGO는 SE3, mono는 창만 Sim3, 관성은 게이트+사슬 간선 | ORB `OptimizeEssentialGraph(Sim3)`/`4DoF`를 뒤로 미룸([ADR-0013](../adr/0013-loop-closing-se3-pgo-weld-ba.md)); KITTI·EuRoC 게이트는 stereo(-inertial)로 측정 |
| P3 구현: weld BA(질의 주변 시각 LBA)로 loose BA·GBA 대체 | OV²SLAM looseBA(전역 5회)·ORB GBA 대신 융합된 covisibility가 두 구간을 잇는 국소 BA; `loop.global_ba` 예약 |
| P3 구현: 병합 좌표계는 IMU 초기화된 쪽 | 중력 정렬(−z) 유지 — 관성 LBA·예측이 고정 중력 벡터를 쓰므로 정렬 안 된 좌표계로 옮기면 깨진다 |
| 단일 KFDB(전 맵), 재측위 전 맵 대상 | `분석/01` §4·§5(ORB reloc active 맵 한정 — 개선) |
| id 기반 참조 + tombstone/epoch | ORB_SLAM3_Modern OWNERSHIP.md(shared_ptr 이행 경험), VIGIL 원칙 "epoch 기반 회수" |
| 이벤트(`PosesCorrected` 등) | `분석/07` §7.1(보정 이벤트 부재가 dense mapping의 장애) |
| 직렬화 항목(IMU 상태·원시 측정 포함, KFDB 재구축) | `분석/01` §5 |
| 워커 3 + LatestQueue + Offline 동기 모드 | `분석/03` §4·§5, `cuvslam_design_analysis.md` §7 |
| Config 1회 검증·setter 없음·internals 분리·에러 3분류 | `cuvslam_design_analysis.md` §4, VIGIL 원칙 "호스트를 죽이지 않음" |
| 서브모듈 shallow 태그 고정, 단일 라이브러리 | 사용자 지시, `cuvslam_design_analysis.md` §11, ORB_SLAM3_Modern R2/R3(Eigen 5 + g2o 20241228 검증) |
| Ceres MINIGLOG/EIGENSPARSE, OpenCV BUILD_LIST 트리밍 | `VINS_Remastered/cmake/Dependencies.cmake` |
| `-march=native` 금지 | `분석/02` §7.1(OV²SLAM), ORB_SLAM3_Modern R1, cuVSLAM 설계 분석 §9.5 |
| 공개 헤더 POD·pimpl, C ABI 후순위 | VIGIL architecture.md §3 |
| dense mapping은 훅만 | `분석/07` §7.1 `[INFER]`(keyframe-anchored submap — 외부 지식, 미검증) |
| KF 시점 검출을 프론트엔드로 이동 | P0 구현: Realtime에서 트랙 id가 한 KF 늦게 생기는 문제(설계 02 §2.7 DIFF) |
| 로컬맵 매칭 = 트랙 alias(병합 대신) | `TrackId ≡ MapPointId` 규약(`분석/06` §6.4 OV²SLAM lmid)과 양립하는 유일한 병합 수단; MH01 실측 0.047 → 0.032 m |
| 단안 초기화 E는 5-pt USAC_ACCURATE | 단위 테스트 실측: 8-pt/5-pt RANSAC 최소표본 해의 회전 오차 ~0.4°(0.2 px 잡음) → 삼각측량 오차 수십 cm |
| KF culling 기본 OFF | MH01 stereo 실측(670→28 KF, ATE 0.032→0.257 m), OV²SLAM도 KF culling 없음(`분석/02` §3) — [ADR-0009](../adr/0009-keyframe-culling-off-for-klt.md) |
| LBA robust 비용 발산 가드 + depth 사전 필터 | MH01 실측: χ² `inf`로 가드 무력화 → 프레임 1380 추적 상실; Ceres 초기 평가 실패 시 LBA 전체 무효(`Initial residual and Jacobian evaluation failed`) |
| 장소 인식 = HBST 온라인 트리(어휘 파일 없음) | clean-room(ORB 어휘 파일은 GPL 저장소 산출물), Schlegel & Grisetti HBST 2018; V101 실측 99.3% — [ADR-0010](../adr/0010-hbst-online-place-recognition.md) |
| 직렬화는 Sophus 원시 저장 경유 | 쿼터니언 재정규화가 LSB를 바꿔 바이트 동일성 실패(라운드트립 테스트 실측) |
| 뷰어 = Rerun(공개 API 어댑터), Pangolin 비채택 | 도커 전용 워크플로우에서 X11/GLX 불필요, `.rrd` 기록·재생, iOS/Jetson 헤드리스 — [ADR-0011](../adr/0011-rerun-viewer.md) |
| 재측위 inlier 임계 20 | V101 실측: 30에서 52.7%(실패 68건 중 64건이 24-27 inlier), 20에서 99.3%·오탐 0 |
| RGB-D 깊이 = 가상 우측 관측(u_r) | `분석/01` §2.7(ORB-SLAM2 RGB-D u_r), fr1_desk 실측 0.080 → 0.030 m |
| KLT 프리셋은 데이터셋별(창·레벨·fb) | fr1_desk 실측: 9/3/0.5 → 프레임당 50% 손실·2회 상실, 21/4/1.5 → 상실 0 |
| 단안 KF culling ON | MH01 mono 실측 0.230(OFF) vs 0.142(ON) vs 0.204(min_other_observers 5) — ADR-0009 보강 |
| 재측위 유예 5프레임 | 한 프레임 실패는 P3P 재시드로 회복; 즉시 재측위는 트랙을 줄인다(설계 02 §4) |
| IMU preintegration/factor 수식 | `분석/01` §2.8(IntegrateNewMeasurement, 1차 bias 보정), `분석/10` §7(EdgeInertial/GS Jacobian); 우측 곱 규약으로 재유도해 수치미분 검증 |
| 관성 LBA 경계 엣지 ×1e-2 + Huber √16.92 | `분석/10` §43(LocalInertialBA 경계 다운웨이트); MH01 실측 0.127 → 0.060 m |
| 관성 LBA 창 10/25 선택 | ORB `bLarge`(inlier > 100 stereo / 75 mono); MH01 실측 창 10: 0.127, 20: 0.101, 25: 0.066 |
| 프레임 pose-inertial 최적화 기본 OFF | MH01 실측 A 0.060 / B 0.070 vs 시각+IMU 예측 0.048 — [ADR-0012](../adr/0012-frame-inertial-optimization-off.md) |
| IMU 초기화 후 누산기 재적분(리셋 아님) | P2 실측: 리셋하면 KF→프레임 구간 소실로 초기화 직후 추적 상실·새 맵 |
| culling 시 preintegration 병합 + 간격 0.5/3 s | `분석/01` §2.8(`mergePrevious`), ORB KeyFrameCulling 관성 조건; mono-inertial 실측(병합 없이 1.38 m) |
