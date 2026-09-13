# 09 — 로드맵과 게이트

> 각 단계는 **완료 조건(게이트)**으로 정의한다. 게이트는 Offline 모드(결정적)에서 측정하고, 실시간 모드 회귀는 별도 항목으로 둔다(cuVSLAM 교훈: 평가는 전부 Offline, 실시간은 어디서도 측정되지 않았음).

| 단계 | 범위 | 게이트 |
|---|---|---|
| **P-skel (완료, 본 세션)** | 설계 문서 11편, 저장소·서브모듈 11종 태그 고정, CMake/프리셋/CI/Docker, 공개 API, core/sensors/map/backend graph·Ceres 어댑터·marginalizer·motion-only BA, frontend 인터페이스·상태기계·KF 정책·KLT/격자 검출(OpenCV), 단위 테스트 | `cmake --preset macos-release` configure·build 통과, 단위 테스트 전부 통과 |
| **P0 — mono/stereo VO(Ceres) (완료, 2026-09-14)** | `MonoTwoViewInitializer`(H/E 병렬, 5-pt USAC), `DepthInitializer`, E-RANSAC 필터, `KltStereoMatcher`, 프론트엔드 KF 시점 격자 검출, Mapper(기술·temporal 삼각측량·`LocalMapMatcher`+트랙 alias·MP culling), `CovisibilityWindow` LBA + 선점/시간 상한/robust 발산 가드/depth 사전 필터, `KeyframeCuller`(구현·**기본 OFF**, ADR-0009) + culled pose 체인, `trajectory()` KF 상대 궤적, 데이터셋 러너(EuRoC·KITTI odometry, calib 자동 반영) + `evaluate_ate.py`(EuRoC/KITTI GT, SE3/Sim3) | **달성**: MH01 stereo Offline ATE **0.032 m**(게이트 ≤ 0.06), 프론트엔드 **3.4 ms/frame**(도커·M4), 2회 실행 궤적 SHA 동일. 부가 실측: MH01 mono Sim3 ATE 0.142 m(scale 3.7, 루프 없음), KITTI 00 stereo ATE 4.99 m(3.7 km, 루프 클로징 없음; P3 게이트 1.5 m는 루프 후 목표) |
| **P1 — RGB-D, 재측위, 맵 저장/로드** | `DepthImageSource`, ORB@KF + `BowPnpRelocalizer`, `IPlaceRecognizer` DBoW 어댑터(어휘 파일) 또는 온라인 어휘 결정, cereal 직렬화(안정 id, IMU 상태), `MapService::load/localize`, KannalaBrandt8 | TUM fr1_desk RGB-D ATE ≤ 0.05 m; 저장→로드→재측위 성공률 EuRoC V1_01 ≥ 90%; 라운드트립 바이트 동일 |
| **P2 — IMU** | `Preintegrator`, `ImuFactor`(Jacobian 테스트), `ImuPredictor`, `PoseInertialOptimizer`(prior via `Marginalizer`), `ImuInitializer` 상태기계(staged prior + scale 분산), `TemporalWindow` 관성 LBA, stereo/rgbd scale 고정 | EuRoC MH01 stereo-inertial ATE ≤ 0.05 m(ORB 실측 0.041 m), mono-inertial ≤ 0.08 m; 초기화 시간 ≤ 15 s; 관성 상실 연명 5 s 테스트 |
| **P3 — 루프 클로징·다중 맵** | `LoopVerifier`(5단+3표), `Sim3/Se3/Yaw4Dof Corrector`, `PoseGraphOptimizer`(essential graph), `LooseBA`, 선택적 GBA(지연 적용), `MapStore.merge`(rigid snap / welding), `PosesCorrected` 이벤트 | KITTI 00 stereo ATE ≤ 1.5 m(ORB 1.2 m), EuRoC MH05 루프 후 개선 확인, LOST→새 맵→병합 시나리오 통합 테스트, 장시간 루프 시퀀스에서 메모리 단조 증가 없음 |
| **P4 — 어댑터·플랫폼** | `G2oSolver`·`GtsamSolver` 어댑터 + 등가성 봉투 테스트, Jetson Orin Docker 실측, iOS XCFramework, C ABI, Python 바인딩(nanobind), 다중 카메라(N>2) FIG식 교차 추적 | 세 어댑터 GT 복구 동일 정확도; Orin 프레임당 ≤ 15 ms; iOS 빌드 통과 |
| **P5 — 실시간 하드닝** | Realtime 모드 회귀 측정(프레임 drain·retention), TSAN 클린, 프로파일 기반 최적화(KLT GPU 옵션, Ceres 대신 자체 Schur SBA 검토), dense mapping 훅 예제 | Realtime ATE가 Offline 대비 +20% 이내(ORB의 5배 열화 반면교사), TSAN 데이터 레이스 0 |

### P0 실측 요약 (Offline, docker-release, Apple M4 컨테이너)
| 시퀀스 | 모드 | 프레임/포즈 | KF | MP | 프론트엔드 | Mapper | LBA | ATE RMSE |
|---|---|---|---|---|---|---|---|---|
| EuRoC MH01 | stereo | 3682 / 3681 | 673 | 15.8k | 3.4 ms | 2.6 ms/KF | 115 ms/KF | **0.032 m** (SE3) |
| EuRoC MH01 | mono | 3682 / 3659 | 292 | 11.1k | 2.7 ms | 1.4 ms/KF | 33 ms/KF | 0.142 m (Sim3, s=3.70) |
| KITTI 00 | stereo | 4541 / 4540 | 1580 | 77.9k | 4.9 ms | 2.6 ms/KF | 67 ms/KF | 4.99 m (SE3) |

P0에서 확정된 설계 편차: 검출은 프론트엔드 KF 시점([02](02-frontend.md) §2.7), 점 파라미터화는 XYZ([03](03-backend.md) §2), KF culling 기본 OFF([ADR-0009](../adr/0009-keyframe-culling-off-for-klt.md)), Offline 실행 모드가 결정성의 근거([ADR-0007](../adr/0007-realtime-offline-execution-modes.md)).

## 리스크와 대응
| 리스크 | 대응 |
|---|---|
| Eigen 5 + GTSAM 4.3a2 미검증 | GTSAM 어댑터는 옵션 OFF 기본, P4에서 검증. 실패 시 GTSAM 4.2.0 + `GTSAM_USE_SYSTEM_EIGEN`으로 대체 핀 |
| Ceres 2.2.0의 Eigen 5 경고/오류 | Homebrew 조합 선례 있음. 문제 시 Eigen 3.4.1 핀으로 후퇴(ADR 기록) |
| OpenCV 서브모듈 빌드 시간(~10분) | 데스크톱/CI는 시스템 OpenCV 우선(`auto`), 서브모듈은 iOS·재현 빌드 전용 |
| KLT 프론트엔드의 실외/저텍스처 취약(VIGIL KITTI 실측) | `IFeatureTracker` 전략으로 descriptor 매칭 추적기를 P4에 추가, 데이터셋별 프리셋 |
| 라이선스 미결정 | clean-room 유지 중(GPL 코드 미복사), LICENSE 파일은 결정 후 추가([ADR-0008](../adr/0008-license-clean-room-pending.md)) |
| KLT 파이프라인에서 KF culling 불가 → 장시간 세션 메모리 | P3 루프 클로징과 함께 descriptor 기반 로컬맵 재관측을 프레임 경로에 넣은 뒤 재평가; 그 전에는 `map.max_keyframes_per_map`(예약)로 상한 |
