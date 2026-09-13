#!/usr/bin/env python3
"""Generates viz/mvslam.rbl — the Rerun layout for recordings with application id `mvslam`
(src/platform/viz/rerun_sink). Run with the Python SDK of the same Rerun version as the C++ SDK:
  docker run --rm -v "$PWD/viz:/viz" python:3.12-slim sh -c "pip install -q rerun-sdk==0.37.2 && python3 /viz/make_blueprint.py /viz/mvslam.rbl"
Load it in the viewer together with the stream: `rerun --port 9876 viz/mvslam.rbl`.
"""
import sys
import rerun.blueprint as rrb

bp = rrb.Blueprint(
    rrb.Horizontal(
        rrb.Spatial3DView(name="맵 · 파랑 프러스텀=키프레임 · 회색=맵 포인트 · 빨강=궤적 · 큰 프러스텀=현재 카메라", origin="/world", contents="/world/**"),
        rrb.Vertical(
            rrb.Spatial2DView(name="카메라0 · 초록=3D 트랙(맵 포인트) · 노랑=2D 트랙(삼각측량 대기) · 꼬리=KLT 이동(10프레임)", origin="/world/cam/image", contents="/world/cam/image/**"),
            rrb.TimeSeriesView(name="프론트엔드 track() ms · 3D 트랙 수 · 백엔드 mapper/LBA ms(KF당 평균)", origin="/stats", contents="/stats/**"),
            rrb.TextLogView(name="이벤트 (frontend / mapper / optimizer)", origin="/events", contents="/events/**"),
            row_shares=[3, 2, 2],
        ),
        column_shares=[3, 1],
    ),
    rrb.BlueprintPanel(state="collapsed"),
    rrb.SelectionPanel(state="collapsed"),
    rrb.TimePanel(state="expanded"),
    auto_views=False,
)
bp.save("mvslam", sys.argv[1] if len(sys.argv) > 1 else "mvslam.rbl")
print("wrote", sys.argv[1] if len(sys.argv) > 1 else "mvslam.rbl")
