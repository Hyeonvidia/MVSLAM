# Contributing

## 코드 규약
- C++20, `CXX_EXTENSIONS OFF`. 포맷은 `.clang-format`, 정적 검사는 `.clang-tidy`.
- 파일 = 클래스 1개, `snake_case.hpp/.cpp`, 클래스 `PascalCase`, 인터페이스 `I` 접두, 멤버 `snake_case_`.
- 헤더 상단 주석 3줄: 역할 / 스레드 소유 / 불변식.
- 서드파티 헤더 include 위치: OpenCV → `src/frontend/opencv/`, `src/backend/mapper/`, `apps/`; Ceres/g2o/GTSAM → `src/backend/solvers/<lib>/`; cereal → `src/map/serialization/`; yaml-cpp → `src/api/config.cpp`.
- 공개 헤더(`include/mvslam/api`)에는 표준 라이브러리 타입만.

## 규칙
1. 모드 분기(`sensor == …`)는 `pipelines/PipelineFactory`에만.
2. 공유 상태는 `LatestQueue`/`std::atomic`/`stop_token`. 폴링 sleep 금지, `detach` 금지.
3. Jacobian이 있는 factor는 `tests/numeric_jacobian.hpp`로 수치 대조 테스트를 동반한다.
4. 튜닝 상수는 `Config`(공개) 또는 `internals`(전문가)로. 소스 하드코딩 금지.
5. 서브모듈 핀 변경은 `scripts/bump_submodule.sh` + `docs/adr/`.
6. 커밋 메시지: Conventional Commits (`feat(frontend): …`, `build(deps): …`).
7. 리뷰 체크리스트: `docs/design/08-design-patterns.md` 하단.

## 빌드·테스트
호스트 빌드 금지. `scripts/docker.sh build [preset]`(configure+build+ctest, 기본 `docker-release`), `scripts/docker.sh tsan`. 통합 게이트는 Offline 모드(`execution: offline`)에서 실행한다(`docs/design/09-roadmap.md`).
