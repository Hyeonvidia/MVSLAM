# ADR-0013 루프 클로징은 SE3 essential-graph PGO + weld BA로 시작한다 (Sim3/4-DoF PGO·GBA 보류)
- 상태: 채택 (2026-09-14)
- 맥락: 설계(03 §5)는 모드별 보정기(Sim3/SE3/Yaw4DoF)와 같은 종류의 PGO factor, OV²SLAM식 loose BA, ORB식 지연 GBA를 두었다. P3에서 stereo·RGB-D·관성 맵이 우선 게이트(KITTI 00, MH05)이고, 프레임워크의 `Variable`은 SE3 pose까지만 정의돼 있다(Sim3 변수·manifold를 Ceres 어댑터에 추가해야 Sim3 PGO가 가능).
- 결정:
  1. **PGO는 SE3 한 종류**(`RelativePoseSE3Factor`, 해석 Jacobian). 시각 전용 mono는 질의 창만 Sim3(3-D/3-D 거리비 중앙값 scale)로 보정하고 PGO는 SE3로 돌린다 — scale 불연속은 창 경계 병진 오차로 흡수된다. 관성 맵은 Yaw-4DoF factor 대신 roll/pitch 게이트(≤ 0.02 rad) + prevKf 사슬 간선으로 근사한다.
  2. **weld BA**: PGO 뒤 질의 KF 주변의 시각 LBA(융합으로 covisibility가 old 구간까지 이어짐)로 loose BA·GBA를 대체한다. 관성 맵은 다음 KF의 관성 LBA에 맡긴다. `loop.global_ba`는 예약.
  3. **병합은 rigid(Sim3) 스냅 + 융합 + weld BA**, essential graph 없음. 좌표계는 IMU 초기화된 쪽이 이긴다(중력 −z 유지).
  4. **HBST 질의는 제외 KF 항목을 잎 탐색에서 건너뛴다**(질의 KF 자신이 색인에 있으므로).
  5. 보정이 무시할 만하면(병진 < 1 cm, 회전 < 0.05°) PGO를 생략하고 융합만 한다(재방문 구간에서 10 KF마다 반복 검출됨).
- 실측(Offline, docker-release, M4): KITTI 00 stereo ATE **1.04 m**(같은 코드로 루프 없이 5.68 m, P0 시점 4.99 m; 게이트 ≤ 1.5 m, 루프 20회), MH05 stereo-inertial **0.078 m**(루프 없이 0.142 m), MH05 stereo 0.103 m(0.232 m), MH01 mono Sim3 **0.106 m**(0.142 m; scale 보정 0.93~1.00), MH01 stereo 5 s 카메라 단절(IMU 없음): 새 맵 3 KF 뒤 old 맵으로 병합, 단절 제외 ATE 0.028 m. EuRoC 설정 파일 기본은 `execution: realtime`이므로 게이트는 `--execution offline`으로 측정(Realtime은 비결정적: 같은 MH05 stereo-inertial이 0.09~0.20 m).
- 결과·후속: mono의 큰 scale drift(장시간 mono 시퀀스)와 관성 맵의 yaw-only 보정은 Sim3/4-DoF PGO가 필요하다 — `Variable::Sim3` + Ceres manifold + `RelativePoseSim3Factor`(수치 Jacobian으로 시작 가능)를 P4 이후 항목으로 둔다. GBA는 KITTI 급 시퀀스에서 ATE 여유(1.04 vs 1.5 m)가 있어 보류.
