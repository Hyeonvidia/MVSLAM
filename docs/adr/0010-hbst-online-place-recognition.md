# ADR-0010 장소 인식은 어휘 파일 없는 온라인 HBST 트리를 기본으로 한다
- 상태: 채택 (2026-09-14)
- 맥락: 재측위·루프 후보 제안에는 KF 데이터베이스가 필요하다. ORB-SLAM 계열은 DBoW2 + 사전 학습 ORB 어휘(145 MB 텍스트, GPL 저장소 산출물)를 쓴다. 본 프로젝트는 clean-room을 유지하고([ADR-0008](0008-license-clean-room-pending.md)) 어휘 파일 배포·로딩(수 초)을 피하려 한다.
- 결정: `IPlaceRecognizer`의 기본 구현을 `HbstPlaceRecognizer`(Hamming Binary Search Tree, Schlegel & Grisetti 2018)로 한다. KF의 ORB descriptor를 잎 ≤ 512개의 이진 트리에 온라인 삽입(잎이 차면 최균형 비트로 분할), 질의는 descriptor별로 잎을 brute-force해 최근접 KF에 투표. DBoW 어댑터는 `PlaceRecognizerType::DBoW`로 자리만 두고(P3, 사용자가 어휘를 제공할 때), `loop.recognizer: incremental`을 모든 기본 설정에 넣는다.
- 결과: V1_01 stereo 맵(535 KF, 16만 descriptor)에서 무상태 재측위 146/146 성공(허용오차 내 99.3%), 질의 ~70 ms(코너 검출·ORB 포함). 트리 구조는 삽입 순서에 의존하므로 Offline 모드에서 결정적이며, 저장하지 않고 로드 시 KF descriptor로 재구축한다. descriptor 단위 재현율은 근사(분할 비트가 뒤집히면 다른 잎)지만 영상 단위 투표로 보상된다.
