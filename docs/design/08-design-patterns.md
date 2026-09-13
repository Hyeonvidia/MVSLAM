# 08 — 디자인 패턴 카탈로그: 어디에, 왜

> 목적: 다른 개발자가 "이 클래스는 무슨 역할이고 왜 이렇게 생겼는가"를 패턴 이름 하나로 파악하게 한다. 각 항목은 (패턴 → 적용 위치 → 해결하는 문제 → 반면교사)로 적는다. 반면교사는 전부 `분석/`에서 실측한 것이다.

| # | 패턴 | 적용 위치 | 해결하는 문제 | 반면교사(피한 것) |
|---|---|---|---|---|
| 1 | **Facade** | `mvslam::System`(`include/mvslam/api/system.hpp`) | 파이프라인 전체를 3개 메서드(`pushImu`, `track`, `mapService`)로 노출. 내부 스레드·맵·솔버를 숨김 | ORB-SLAM3 `System`이 Tracking/LocalMapping/LoopClosing 포인터를 서로에게 배선하며 공개 헤더에 내부 타입을 노출 |
| 2 | **Pimpl** | `System::Impl`, 공개 헤더는 POD·표준 컨테이너만 | ABI 안정, 서드파티 헤더(Eigen/OpenCV/Ceres) 비전파 — iOS·Python 바인딩의 전제 | cuVSLAM `cuvslam2.h`가 "std::string 금지" 규칙을 문서에 두고도 `std::function`·`shared_ptr`를 노출한 사례 |
| 3 | **Factory** | `pipelines::PipelineFactory::build(Config)` | `SensorMode × InertialMode × ExecutionMode` 조합을 한 곳에서 전략 객체로 합성. 모드 `switch`는 여기 한 곳뿐 | ORB-SLAM3 `Tracking.cc`의 `if(mSensor==…)` 수십 곳(6 모드 조건 분기가 4,126줄에 산재) |
| 4 | **Strategy** | `IFeatureTracker`, `IFeatureDetector`, `IDescriptorExtractor`, `IOutlierFilter`, `IPosePredictor`, `IPoseOptimizer`, `IKeyframePolicy`, `IInitializer`, `IDepthSource`, `ICameraModel`, `IWindowSelector`, `ILoopCorrector`, `IPlaceRecognizer`, `ISolver` | 알고리즘 교체(KLT↔descriptor, Ceres↔g2o↔GTSAM, Sim3↔SE3↔4DoF, DBoW↔온라인 어휘)를 재컴파일 없이 설정으로 | OV²SLAM `Optimizer`가 Ceres에 직접 결합, ORB `Optimizer.cc` 5,590줄이 g2o 타입에 직접 결합 |
| 5 | **Adapter** | `backend/solvers/ceres/CeresSolver`(`FactorCost`, `Se3Manifold`), 예정 `G2oSolver`, `GtsamSolver` | 라이브러리 API를 `Problem/IFactor` 추상으로 변환. 잔차·Jacobian은 어댑터 밖(`IFactor`)에 있어 세 라이브러리가 같은 수식을 공유 | DM-VIO가 GTSAM 타입에 강결합되어 4.2a6 고정·모바일 이식 난항(`분석/08` §3) |
| 6 | **Template Method** | `frontend::VisualFrontend::process()` | 프레임 처리 순서(전처리→예측→추적→필터→최적화→상태→KF)는 고정, 각 단계는 주입된 전략이 수행 | OV²SLAM `trackMono`가 단계와 구현을 한 함수(1,216줄 파일)에 혼합 |
| 7 | **State** | `frontend::TrackingStateMachine`, `backend::imu::ImuInitializer` | 상실/연명/새 맵 전이와 IMU 초기화 단계(Uninitialized→Coarse→Initialized→Refine1/2)를 명시적 전이표로 | ORB `Track()` 안의 `mState` 분기 + `mTinit` 시간 비교가 흩어져 있어 55/65/75 s 분기가 도달 불가인 것을 아무도 몰랐던 사례(`분석/00`) |
| 8 | **Decorator** | `ClosePointRulePolicy`, `InertialTimeRulePolicy`가 `ParallaxTrackRatioPolicy`를 감쌈 | 모드별 KF 규칙 추가를 상속 폭발 없이 조합 | ORB `NeedNewKeyFrame`의 c1a/c1b/c1c/c2/c3/c4 조건이 센서 플래그와 뒤섞인 단일 함수 |
| 9 | **Observer** | `map::MapEventBus`, `MapService::subscribe` | dense mapping·시각화·로깅이 맵 보정(`PosesCorrected`)을 구독. 구독자 없으면 비용 0 | ORB-SLAM3에 pose 보정 이벤트가 없어 dense 모듈이 KF pose를 폴링해야 함(`분석/07` §7.1) |
| 10 | **Command** | Loop 워커의 `CommandQueue`(save/load/localize) | 비동기 맵 명령을 KF 처리와 같은 큐에서 순서대로, 콜백에 `Result` | ORB `SaveAtlas`가 `Shutdown` 안에서만 동기 실행 |
| 11 | **Producer/Consumer + Latest-only queue** | `core::LatestQueue`, `core::Worker` | 생산자 비블록, 최신만 처리, 버린 항목 병합 훅 | ORB·OV²SLAM의 non-atomic bool + `usleep` 폴링(`분석/03`) |
| 12 | **Null Object** | `map::NullPlaceRecognizer`, LC 비활성 | 루프 클로징 off를 `if(enabled)` 산재 없이 표현 | OV²SLAM이 iBoW-LCD `.so` 존재 여부로 LC를 조용히 끄는 빌드 함정 |
| 13 | **Dependency Injection** | `FrontendComponents`, `IClock`, `ISolver` 주입, `IPoseOptimizer`를 frontend가 선언하고 backend가 구현 | frontend↔backend 순환 의존 제거, 테스트에서 가짜 구현 주입 | ORB Tracking이 Optimizer·LocalMapping·LoopClosing을 직접 호출 |
| 14 | **RAII** | `Subscription`, `ScopedTimer`, `std::jthread`, `std::unique_lock`, `Map::withWriteLock`의 epoch bump | 해제 누락·detach 스레드·락 누수 제거 | ORB `Shutdown` join 주석 처리, GBA detach, `mpcpi` 누수 |
| 15 | **Immutable Config / Validate-once** | `Config::validate()`, setter 없음, `effectiveConfigYaml()` | 모순 설정을 생성자에서 거부, 재현성 | ORB 신·구 파서 2벌 공존, `mnFramesToResetIMU` 미초기화 UB |
| 16 | **Strong typedef ids** | `core::Id<Tag>` | KF/MP/Map/Track/Variable id 혼용을 컴파일 타임에 차단, 직렬화·세션 재개 안전 | ORB 포인터 그래프(PreSave/PostLoad 변환 코드 수백 줄) |
| 17 | **Tombstone + Epoch reclamation** | `Map::removeKeyFrame/reclaim` | 무락 리더가 든 객체를 안전하게 지연 해제하면서 메모리 상한 유지 | ORB soft-delete 무회수(`RemoveBadMaps` delete 주석) |
| 18 | **Copy-out / Write-back** | `Map::withReadLock` 스냅샷 → 락 밖 최적화 → `withWriteLock` 반영 | 긴 최적화 동안 락을 잡지 않음 | ORB도 같은 패턴(`mMutexMapUpdate`)이나 객체별 mutex 7종과 락 순서 규약이 추가로 필요했음 |
| 19 | **Value-typed Variable/Factor + IRLS** | `graph::Variable`, `graph::IFactor::loss` | 솔버 없이도 `Problem::linearizeDense`로 H/b를 만들 수 있어 marginalization·테스트가 독립 | 세 라이브러리 모두 marginalization API 부재(`분석/10` §1) |

## 안티패턴 체크리스트 (코드 리뷰용)
- [ ] 모드 조건문(`if (cfg.sensor == …)`)이 `PipelineFactory` 밖에 있다 → Strategy/Decorator로 옮긴다.
- [ ] 공유 상태를 `bool` 플래그로 폴링한다 → `LatestQueue`/`std::atomic`/`stop_token`.
- [ ] 스레드를 `detach`한다 → `Worker`(jthread).
- [ ] 라이브러리 타입(Ceres/g2o/GTSAM/OpenCV)이 `solvers/`·`frontend/opencv/`·`apps/` 밖에 나타난다.
- [ ] 튜닝 상수가 소스에 하드코딩된다 → `Config`/`internals`.
- [ ] 한 파일이 1,000줄을 넘거나 한 클래스가 두 스레드에서 쓰기를 받는다.
- [ ] Jacobian을 가진 factor에 수치 미분 테스트가 없다.
