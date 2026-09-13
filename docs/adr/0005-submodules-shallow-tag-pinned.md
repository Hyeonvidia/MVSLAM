# ADR-0005 서드파티는 shallow git 서브모듈 + 릴리스 태그 고정
- 상태: 채택 (2026-09-13)
- 맥락: 사용자 선호(서브모듈), cuVSLAM의 "의존성 전부 고정" 원칙, VINS_Remastered는 FetchContent를 썼음.
- 결정: `third_party/`에 11종을 `shallow = true`로 두고 태그 고정(`07-build-platforms.md` 표). 갱신은 `scripts/bump_submodule.sh` + ADR. 단일 Eigen(5.0.1)을 `CMAKE_FIND_PACKAGE_REDIRECTS_DIR`로 모든 서브프로젝트에 강제.
- 핀: eigen 5.0.1 · sophus 1.24.6 · ceres-solver 2.2.0 · g2o 20241228_git · gtsam 4.3a2 · opencv 4.14.0 · yaml-cpp 0.9.0 · spdlog 1.17.0 · googletest 1.18.0 · cereal 1.3.2 · benchmark 1.9.5.
- 결과: 최초 체크아웃 ≈ 400 MB. GTSAM 4.3a2 + Eigen 5 조합은 미검증(옵션 OFF 기본).
