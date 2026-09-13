# ADR-0009 KLT 파이프라인에서는 키프레임 culling을 기본 비활성화한다
- 상태: 채택 (2026-09-14)
- 맥락: ORB-SLAM3의 KF culling 규칙(점의 90%가 다른 ≥ 3 KF에서 관측되면 잉여)은 descriptor 기반 로컬맵 추적이 매 프레임 모든 로컬 MP를 재관측한다는 전제 위에 있다. 본 프로젝트의 프론트엔드는 OV²SLAM 계보의 KLT 추적이라 한 MP의 관측은 그 KLT 트랙이 살아 있는 연속 KF(EuRoC에서 10여 개)로 한정되고, KF 밀도가 곧 새 점 공급(스테레오 매칭·temporal 삼각측량)과 LBA 제약의 밀도다.
- 실측(EuRoC MH01 stereo, Offline, `KeyframeCuller` ORB 규칙 + 참조 KF 상대 pose 체인 적용): culling ON → 670 KF 중 28 KF, 2.1k MP, ATE 0.257 m; OFF → 673 KF, 15.8k MP, ATE 0.032 m. `kf_cull_min_other_observers`를 올려도 KLT 관측 구조상 대부분의 KF가 잉여로 판정된다.
- 결정: `backend.kf_culling` 기본값 false. `KeyframeCuller`·`Map::CulledRecord`·`resolvePose` 체인·관측 < 2 MP 동반 제거는 구현·테스트된 채로 유지한다(ORB 의미론 그대로). OV²SLAM 역시 KF culling을 하지 않는다.
- 결과: 맵 KF 수는 세션 길이에 비례해 증가한다(MH01 3분에 673 KF, 메모리 상 문제 없음). 장시간 세션 상한은 P3에서 descriptor 기반 로컬맵 재관측을 프레임 경로에 추가한 뒤 culling을 재평가하거나 `map.max_keyframes_per_map`으로 둔다.
