# 06 — 동시성, 실행 모드, 결정성

> 반면교사: ORB-SLAM3·OV²SLAM은 non-atomic bool + `usleep` 폴링, detach된 GBA 스레드, join 주석 처리(`분석/03` §5). 채택: cuVSLAM의 "생산자 절대 비블록 + 워커 큐 + Offline 동기 모드", OV²SLAM의 "drain·선점·시간 상한·역압".

## 1. 스레드 모델
| 스레드 | 생성 | 역할 | 입력 | 출력 |
|---|---|---|---|---|
| 호출 스레드 | 사용자 | `pushImu`, `track`(프론트엔드 전체), `trajectory` | `FrameInput`, `ImuSample` | `TrackResult`, `KeyframeRequest` 큐 push |
| `MapperWorker` | `System` 생성 시(Realtime) | KF 측정 추출·깊이·삼각측량·로컬맵 매칭 → `Map` write-back | `LatestQueue<KeyframeRequest>` | `LatestQueue<KeyframeId>`(Optimizer) |
| `OptimizerWorker` | 〃 | IMU 초기화 단계, LBA, MP/KF culling | `LatestQueue<KeyframeId>` | `LatestQueue<KeyframeId>`(Loop), `optimizer_busy` atomic |
| `LoopWorker` | 〃(`loop.enable && recognizer != none`) | place recognition, 검증, 보정, PGO, weld BA, 병합(P3; 전용 P3P·MotionOnlyBa·LBA 인스턴스), **커맨드**(save/load/localize) | `LatestQueue<KeyframeId>` + `CommandQueue` | `PosesCorrected`·`LoopClosed`·`MapMerged` 이벤트 |
| (없음) | — | 임시 스레드·detach 금지. GBA도 LoopWorker 안에서 `StopToken`으로 중단 가능한 작업 | | |

`std::jthread` + `std::stop_token`으로 생명주기를 관리한다. `System::~System`은 `request_stop()` → 큐 깨우기 → join. 폴링 sleep 없음: 큐는 `condition_variable_any`로 대기(`stop_token` 연동).

## 2. 큐와 신호
- `LatestQueue<T>`: 용량 1. push는 기존 항목을 **덮어쓰며 병합 훅**을 호출(버린 KF id를 covisibility 합류 목록에 축적 — OV²SLAM `getNewKf` 트릭). 생산자는 절대 블록되지 않는다.
- `CommandQueue`: FIFO(save/load/localize). KF 처리와 같은 워커가 순서대로 처리(cuVSLAM `ICommand`).
- `StopToken`: Mapper가 새 KF를 받으면 Optimizer의 진행 중 BA에 stop 요청(`ISolver`는 반복마다 검사). 로컬맵 매칭도 같은 토큰으로 조기 종료.
- 프론트엔드 역압: `optimizer_busy`(atomic) 및 Mapper 큐 점유가 KF 판정에 반영([02-frontend.md](02-frontend.md) §2.6). Mapper 큐가 이미 차 있으면 mono는 KF를 포기하고 stereo류는 덮어쓴다(ORB 정책의 큐 판).

## 3. 락 규약
| 자원 | 보호 | 규칙 |
|---|---|---|
| `Map` | `std::shared_mutex` 1개 | 읽기: shared lock 아래 **스냅샷 복사** 후 해제. 쓰기: unique lock 아래 짧게(write-back만). 최적화·매칭 연산은 락 밖 |
| `MapStore` | `std::mutex` | 맵 집합·active 포인터. 순서: `MapStore` → `Map` (역순 금지) |
| 큐 | 내부 mutex + cv | |
| 플래그 | `std::atomic` | non-atomic 공유 bool 금지(clang-tidy `concurrency-*` + TSAN 프리셋) |
| 이벤트 콜백 | 락 밖 | 콜백 안에서 `System` API 호출 금지(재진입 방지, 문서화) |
디버그 빌드는 `Map::assertWriteLocked()`로 "쓰기는 write lock 안에서만" 불변식을 검사한다.

## 4. 실시간 정책 (Realtime)
1. **프레임**: `track()`은 프론트엔드 처리를 동기 수행(수 ms)하고 백엔드는 큐에 넘긴다. 입력이 처리보다 빠르면 **호출자**가 프레임을 버린다(`apps/`의 `FrameDropper`가 예시; 코어는 큐를 두지 않아 지연이 쌓이지 않는다).
2. **KF**: `LatestQueue`로 최신만 처리, 버린 KF의 covisibility는 합류.
3. **BA**: 시간 상한(0.2 s) + 선점. Offline에서는 상한 비활성(결정성).
4. **보정 지연**: PGO/GBA 결과는 KF에 적용되고 프레임 궤적은 `T_wk·T_kb`로 자동 갱신. `TrackResult.pose`는 최신 추정이므로 루프 보정 시 **점프**한다(cuVSLAM 계약) — 부드러운 궤적은 `trajectory()`. 보정은 write lock 안에서 `Map::correctionEpoch`을 올리고, LBA(시각·관성)는 스냅샷 때의 epoch과 다르면 write-back을 버린다(보정 전 pose로 계산한 결과가 보정을 되돌리는 것을 막음). Offline(Sync)에서는 mapper → LBA → loop가 프론트엔드 스레드에서 순서대로 실행되므로 겹침이 없다.
5. **retention window**: 보정 결과가 현재 시각보다 `retention_s`(기본 5 s) 이상 오래된 상태를 대상으로 하면 로그만 남기고 폐기하지 않는다 — 맵 보정은 항상 유효. 대신 **로컬맵 스냅샷 epoch**가 오래되면 프론트엔드가 다음 프레임에서 재스냅샷.

## 5. Offline 모드 (결정성)
- 워커를 만들지 않는다. `track()` 끝에서 `Mapper → Optimizer → Loop`를 **같은 스레드에서 순서대로** 실행한다(cuVSLAM `reproduce_mode`/`async_sba=false`).
- RNG는 `std::mt19937_64(seed)`(기본 0)로 주입(RANSAC 전부). OpenCV RANSAC은 `cv::setRNGSeed`로 고정.
- 시간 상한·역압·drain은 비활성(wall-clock을 논리에 쓰지 않음). `IClock`을 주입해 로깅 시간도 데이터 시간으로 통일.
- 솔버 `num_threads = 1`. Eigen 병렬화 OFF.
- 테스트: 같은 시퀀스를 2회 실행해 궤적 파일 SHA-256 동일(자기결정성)을 CI 게이트로.

## 6. 종료와 예외
- `~System()`: stop → 큐 깨움 → join(타임아웃 시 로그 + detach 하지 않고 기다림). 미저장 맵은 경고.
- 워커 루프는 예외를 잡아 로깅하고 다음 항목으로 진행(호스트 프로세스 보호). 연속 N회 예외면 워커를 `degraded` 상태로 표시하고 `stats()`에 노출.

## 7. TSAN/ASAN
- CMake 프리셋 `linux-tsan`, `linux-asan`으로 통합 테스트(EuRoC 짧은 구간)를 돌린다. 데이터 레이스 0이 머지 조건.
