# ADR-0006 공개 API는 POD + pimpl, 서드파티 타입 비노출, C ABI는 후순위 래퍼
- 상태: 채택 (2026-09-13)
- 맥락: VIGIL 원칙(C ABI 경계), cuVSLAM의 ABI-lite 규칙이 지켜지지 않은 사례.
- 결정: `include/mvslam/api`는 표준 라이브러리 타입만. `System`/`MapService`는 pimpl. 이미지는 `ImageView`(비소유). C ABI는 P4에 pimpl 위의 얇은 래퍼로.
- 결과: Swift/Python 바인딩 비용 최소, ABI 안정. 대가: 내부 Eigen 타입과의 변환 코드(`core/math/se3.hpp`).
