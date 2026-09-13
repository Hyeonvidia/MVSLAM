# ADR-0007 실행 모드 Realtime/Offline을 이름 있는 설정으로 두고 Offline은 동기·결정적으로
- 상태: 채택 (2026-09-13)
- 맥락: ORB/OV²SLAM/DM-VIO 모두 결정성이 옵션 뒤에 숨어 있어 공식 평가가 N회 median이다. cuVSLAM은 Offline 모드로 평가한다.
- 결정: `Config.execution`. Offline은 워커를 만들지 않고 `track()` 안에서 Mapper→Optimizer→Loop를 순서대로 실행, 시드 고정, 시간 상한 비활성, 솔버 1스레드. 모순 설정(`Offline` + 멀티스레드)은 `validate()`가 거부.
- 결과: 평가·CI는 전부 Offline. Realtime 회귀는 별도 게이트(P5).
