# 09 — 로드맵과 게이트

> 각 단계는 **완료 조건(게이트)**으로 정의한다. 게이트는 Offline 모드(결정적)에서 측정하고, 실시간 모드 회귀는 별도 항목으로 둔다(cuVSLAM 교훈: 평가는 전부 Offline, 실시간은 어디서도 측정되지 않았음).

| 단계 | 범위 | 게이트 |
|---|---|---|
| **P-skel (완료, 본 세션)** | 설계 문서 11편, 저장소·서브모듈 11종 태그 고정, CMake/프리셋/CI/Docker, 공개 API, core/sensors/map/backend graph·Ceres 어댑터·marginalizer·motion-only BA, frontend 인터페이스·상태기계·KF 정책·KLT/격자 검출(OpenCV), 단위 테스트 | `cmake --preset macos-release` configure·build 통과, 단위 테스트 전부 통과 |
| **P0 — mono/stereo VO(Ceres)** | `TwoViewHFInitializer`, `DepthInitializer`, E-RANSAC 필터, `KltStereoMatcher`, Mapper(검출·기술·삼각측량·로컬맵 매칭), `CovisibilityWindow` LBA + 선점/시간 상한, MP/KF culling(anchor 이전), `trajectory()` KF 상대 궤적, 데이터셋 러너(EuRoC/KITTI) | EuRoC MH01 stereo Offline ATE ≤ 0.06 m(OV²SLAM 실측 0.032 m·ORB stereo 0.038 m 참고), 프레임당 프론트엔드 ≤ 10 ms(M4), 자기결정성(2회 실행 궤적 SHA 동일) |
| **P1 — RGB-D, 재측위, 맵 저장/로드** | `DepthImageSource`, ORB@KF + `BowPnpRelocalizer`, `IPlaceRecognizer` DBoW 어댑터(어휘 파일) 또는 온라인 어휘 결정, cereal 직렬화(안정 id, IMU 상태), `MapService::load/localize`, KannalaBrandt8 | TUM fr1_desk RGB-D ATE ≤ 0.05 m; 저장→로드→재측위 성공률 EuRoC V1_01 ≥ 90%; 라운드트립 바이트 동일 |
| **P2 — IMU** | `Preintegrator`, `ImuFactor`(Jacobian 테스트), `ImuPredictor`, `PoseInertialOptimizer`(prior via `Marginalizer`), `ImuInitializer` 상태기계(staged prior + scale 분산), `TemporalWindow` 관성 LBA, stereo/rgbd scale 고정 | EuRoC MH01 stereo-inertial ATE ≤ 0.05 m(ORB 실측 0.041 m), mono-inertial ≤ 0.08 m; 초기화 시간 ≤ 15 s; 관성 상실 연명 5 s 테스트 |
| **P3 — 루프 클로징·다중 맵** | `LoopVerifier`(5단+3표), `Sim3/Se3/Yaw4Dof Corrector`, `PoseGraphOptimizer`(essential graph), `LooseBA`, 선택적 GBA(지연 적용), `MapStore.merge`(rigid snap / welding), `PosesCorrected` 이벤트 | KITTI 00 stereo ATE ≤ 1.5 m(ORB 1.2 m), EuRoC MH05 루프 후 개선 확인, LOST→새 맵→병합 시나리오 통합 테스트, 장시간 루프 시퀀스에서 메모리 단조 증가 없음 |
| **P4 — 어댑터·플랫폼** | `G2oSolver`·`GtsamSolver` 어댑터 + 등가성 봉투 테스트, Jetson Orin Docker 실측, iOS XCFramework, C ABI, Python 바인딩(nanobind), 다중 카메라(N>2) FIG식 교차 추적 | 세 어댑터 GT 복구 동일 정확도; Orin 프레임당 ≤ 15 ms; iOS 빌드 통과 |
| **P5 — 실시간 하드닝** | Realtime 모드 회귀 측정(프레임 drain·retention), TSAN 클린, 프로파일 기반 최적화(KLT GPU 옵션, Ceres 대신 자체 Schur SBA 검토), dense mapping 훅 예제 | Realtime ATE가 Offline 대비 +20% 이내(ORB의 5배 열화 반면교사), TSAN 데이터 레이스 0 |

## 리스크와 대응
| 리스크 | 대응 |
|---|---|
| Eigen 5 + GTSAM 4.3a2 미검증 | GTSAM 어댑터는 옵션 OFF 기본, P4에서 검증. 실패 시 GTSAM 4.2.0 + `GTSAM_USE_SYSTEM_EIGEN`으로 대체 핀 |
| Ceres 2.2.0의 Eigen 5 경고/오류 | Homebrew 조합 선례 있음. 문제 시 Eigen 3.4.1 핀으로 후퇴(ADR 기록) |
| OpenCV 서브모듈 빌드 시간(~10분) | 데스크톱/CI는 시스템 OpenCV 우선(`auto`), 서브모듈은 iOS·재현 빌드 전용 |
| KLT 프론트엔드의 실외/저텍스처 취약(VIGIL KITTI 실측) | `IFeatureTracker` 전략으로 descriptor 매칭 추적기를 P4에 추가, 데이터셋별 프리셋 |
| 라이선스 미결정 | P0 착수 전 결정(clean-room 유지 시 permissive) |
