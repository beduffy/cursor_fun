from __future__ import annotations

import os
import shlex
import tempfile
from typing import List

try:
    from .ffmpeg_utils import EditSegment, export_edit
    from .models import Clip, Project
except ImportError:  # Fallback for running as plain scripts
    from ffmpeg_utils import EditSegment, export_edit  # type: ignore
    from models import Clip, Project  # type: ignore


def project_to_segments(project: Project) -> List[EditSegment]:
    # Simple compositor: higher track_index renders on top.
    # For MVP we do linearize by track priority and time, blending by top-most only.
    # To keep MVP simple, we will concatenate by increasing timeline time, resolving
    # overlaps by choosing higher track clip.

    # Build a coarse time grid at 0.1s resolution to pick top-most clip
    duration = project.get_timeline_duration()
    if duration <= 0:
        return []

    time_points = []
    t = 0.0
    step = 0.1
    while t < duration - 1e-6:
        time_points.append(round(t, 3))
        t += step
    time_points.append(round(duration, 3))

    def active_clip_at(time_s: float) -> Clip | None:
        candidates = [
            c for c in project.clips.values()
            if (c.timeline_start <= time_s < c.timeline_end)
        ]
        if not candidates:
            return None
        # Higher track_index wins
        return sorted(candidates, key=lambda c: c.track_index, reverse=True)[0]

    # Walk time, collect contiguous spans where the same top-most clip is active
    spans: List[tuple[Clip, float, float]] = []
    current_clip: Clip | None = None
    span_start: float | None = None

    for i in range(len(time_points) - 1):
        t0 = time_points[i]
        t1 = time_points[i + 1]
        clip = active_clip_at((t0 + t1) * 0.5)
        if clip is None:
            # close any active span
            if current_clip is not None and span_start is not None:
                spans.append((current_clip, span_start, t0))
                current_clip = None
                span_start = None
            continue
        if current_clip is None:
            current_clip = clip
            span_start = t0
        elif clip.id != current_clip.id:
            spans.append((current_clip, span_start if span_start is not None else t0, t0))
            current_clip = clip
            span_start = t0
        # else: continue span

    if current_clip is not None and span_start is not None:
        spans.append((current_clip, span_start, duration))

    # Convert spans to EditSegment using each clip's source mapping
    segments: List[EditSegment] = []
    for clip, t0, t1 in spans:
        offset_in_clip = t0 - clip.timeline_start
        seg_in = clip.source_in + max(0.0, offset_in_clip)
        seg_out = seg_in + max(0.0, t1 - t0)
        segments.append(EditSegment(
            source_path=project.sources[clip.source_id].path,
            start_seconds=seg_in,
            end_seconds=seg_out,
        ))
    return segments


def render_project(project: Project, output_path: str) -> None:
    segments = project_to_segments(project)
    export_edit(segments, output_path)


