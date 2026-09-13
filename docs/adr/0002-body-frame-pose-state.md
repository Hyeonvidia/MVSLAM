# ADR-0002 백엔드 pose 상태는 body frame(T_wb) 하나, 카메라는 rig extrinsic으로 파생
- 상태: 채택 (2026-09-13)
- 맥락: ORB-SLAM3는 시각 BA에 카메라 pose 정점(좌측 exp), 관성 BA에 body pose 정점(우측 곱)을 따로 두어 규약이 3종이었다(`분석/05` §3-1). cuVSLAM은 `rig_from_world` 하나다.
- 결정: 모든 `Pose` 변수는 `T_wb`, 증분은 우측 곱 `T·exp([ρ,φ])`. 시각 전용 mono/rgbd에서는 body = camera 0(`T_bc0 = I`). N-카메라는 `CameraRig::T_bc(i)`로 즉시 지원.
- 결과: `ReprojectionFactor` 하나가 mono/stereo/rgbd/multi-cam/inertial 전부를 다룬다. `ImuFactor`와 재투영 factor가 같은 변수를 공유하므로 관성 결합에 변환 factor(DM-VIO `PoseTransformationFactor`)가 필요 없다.
