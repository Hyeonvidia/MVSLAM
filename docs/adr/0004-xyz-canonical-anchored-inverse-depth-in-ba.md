# ADR-0004 점은 XYZ가 정본, anchored inverse depth는 BA 내부 파라미터화
- 상태: 채택 (2026-09-13)
- 맥락: OV²SLAM은 XYZ를 저장하고 BA마다 anchor를 재선정해 ρ를 재계산한다(`분석/02` §3.3). ORB는 `SetWorldPos` 사용처가 20곳(`분석/06` §6.2).
- 결정: `MapPoint::position()`(XYZ)이 정본. `LocalBundleAdjuster`가 anchor(최초 관측 KF, 윈도우 안일 때)를 정해 `AnchoredInvDepthFactor`로 풀고 결과를 XYZ로 환원. anchor KF 삭제 시 `MapPoint::removeObservation`이 다음 관측 KF로 이전. Sim3 보정은 anchor pose 갱신 + ρ/s.
- 결과: 루프/병합/직렬화 코드가 파라미터화를 모른다; BA 문제 크기는 점당 1-DoF.
