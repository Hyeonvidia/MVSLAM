# ADR-0011 뷰어는 Rerun(공개 API 어댑터)으로 하고 Pangolin은 쓰지 않는다
- 상태: 채택 (2026-09-14, 사용자 확정)
- 맥락: 빌드·실행이 도커 전용이라(ADR 없음, 사용자 지시 2026-09-13) X11/GLX 포워딩이 필요한 Pangolin은 macOS 호스트에서 마찰이 크고 iOS·Jetson 헤드리스와도 맞지 않는다. 디버깅용 OpenCV 파일 오버레이(`--overlay-dir`)는 있지만 3D 맵·궤적 검토가 안 된다.
- 결정: Rerun C++ SDK(0.37.2, 릴리스 zip sha256 고정)를 도커 이미지에 설치하고, `src/platform/viz/rerun_sink`를 **공개 API(System·MapService 이벤트·스냅샷)만 쓰는 어댑터**로 둔다. 코어에는 뷰어 코드가 없고 구독자가 없으면 비용 0. 러너 `--rerun` 옵션으로 `.rrd` 파일(도커 안) 또는 호스트 뷰어 실시간 연결(`rerun+http://host.docker.internal:9876/proxy`)을 고른다.
- 결과: 코어 라이브러리 의존성 불변(옵션 `MVSLAM_WITH_RERUN`, 기본 OFF). `.rrd`는 같은 버전의 Rerun 뷰어(`pip install rerun-sdk==0.37.2`)로 연다. MH01 600프레임 스모크에서 `.rrd` 53 MB(이미지 5프레임마다) 생성·`rerun rrd stats` 검증.
