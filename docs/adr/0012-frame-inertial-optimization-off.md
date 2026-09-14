# ADR-0012 관성 모드의 프레임 pose 최적화는 시각 MotionOnlyBa + IMU 예측을 기본으로 한다
- 상태: 채택 (2026-09-14)
- 맥락: 설계(02 §2.4)는 ORB-SLAM3처럼 프레임의 15-D 상태를 IMU factor(+직전 프레임 prior)와 함께 푸는 `PoseInertialOptimizer`를 관성 모드의 기본으로 두었다. CASE A(마지막 KF 고정)와 CASE B(직전 프레임 + dense Schur prior)를 모두 구현해 EuRoC MH01 stereo-inertial(Offline)로 비교했다.
- 실측: 시각 MotionOnlyBa(초기값은 IMU 예측) 0.048 m, CASE A 0.060 m, CASE B 0.070 m. 백엔드(초기화·관성 LBA)는 동일. 프레임 IMU factor의 공분산이 픽셀 잔차보다 훨씬 작아 프레임 pose가 KF/직전 프레임의 속도·bias 오차에 끌려가는 것으로 보이나 원인 분석은 완료하지 못했다(ORB-SLAM3도 MH01에서 stereo-inertial 0.041 m가 stereo 0.038 m보다 나쁘다).
- 결정: `frontend.inertial_pose_optimization` 기본 false. IMU는 (1) 예측 초기값, (2) 시각 실패 시 dead reckoning, (3) 관성 LBA·초기화에 쓰인다. 구현·테스트는 유지하고 저텍스처·고속 시퀀스(V2_03 등)에서 재평가한다.
- 결과: P2 게이트 달성(stereo-inertial 0.048, mono-inertial 0.048 m). 프레임 단위 관성 결합의 이점(빠른 회전·텍스처 부족)은 다른 시퀀스에서 검증되기 전까지 주장하지 않는다.
