# ADR-0003 scale은 log 파라미터(s = exp σ)로만 최적화한다
- 상태: 채택 (2026-09-13)
- 맥락: ORB-SLAM3 `VertexScale`은 승산 갱신(`s·exp(u)`)인데 `EdgeInertialGS` Jacobian은 가산(∂r/∂s)이라 GN 반복이 `s ← s·exp(s*−s)`가 되어 s* ≥ 2에서 발산한다(ORB_SLAM3_Modern P6-3 실측). 단안-관성 초기화가 큰 스케일 오차를 못 잡는 메커니즘이다.
- 결정: `Variable::LogScale`(σ)만 허용, 모든 factor의 scale Jacobian은 `∂r/∂σ = ∂r/∂s · s`. 수치 미분 테스트로 강제.
- 결과: 초기 scale 오차 크기와 무관하게 GN/LM 수렴. stereo/rgbd는 σ 고정(0).
