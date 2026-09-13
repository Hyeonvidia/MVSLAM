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
        rrb.Spatial3DView(name="Map | blue frustums = keyframes, grey = map points, red = trajectory, large frustum = current camera", origin="/world", contents="/world/**"),
        rrb.Vertical(
            rrb.Spatial2DView(name="Camera 0 | green = 3-D tracks (map points), yellow = new 2-D tracks (awaiting triangulation), tails = KLT motion (10 frames)", origin="/world/cam/image", contents="/world/cam/image/**"),
            rrb.TimeSeriesView(name="Timing | frontend track() ms, 3-D tracks, backend mapper / LBA ms per keyframe (avg)", origin="/stats", contents="/stats/**"),
            rrb.TextLogView(name="Events | frontend / mapper / optimizer", origin="/events", contents="/events/**"),
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
