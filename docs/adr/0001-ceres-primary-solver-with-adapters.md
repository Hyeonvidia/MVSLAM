# ADR-0001 Ceres를 1차 솔버로, g2o/GTSAM은 동일 Problem 위의 옵션 어댑터
- 상태: 채택 (2026-09-13)
- 맥락: 사용자 요구 "GTSAM/Ceres/g2o를 서브모듈로 관리". 분석 결과 세 라이브러리 모두 marginalization API가 없고, ORB의 g2o 포크는 upstream 추종이 끊겼으며, DM-VIO는 GTSAM에 강결합되어 모바일 이식이 어려웠다(`분석/10` §8).
- 결정: 잔차·Jacobian은 `backend/graph`(라이브러리 독립)에 두고, 솔버는 `ISolver` 어댑터로 교체 가능하게 한다. Ceres 어댑터를 먼저 구현·검증하고 g2o/GTSAM 어댑터는 P4. 어댑터 간 검증은 GT 복구 정확도(정확 판정) + 어댑터 간 통계 봉투(회귀 지표).
- 결과: 어댑터당 ~200줄; 최적화 함수마다 라이브러리 코드를 복붙하던 ORB 방식(18함수 5,590줄) 대신 문제 조립 코드가 라이브러리와 무관해진다. 대가: `FactorCost::Evaluate`의 값 변환 오버헤드(P5 최적화 항목).
