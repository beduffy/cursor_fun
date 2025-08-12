from __future__ import annotations

import cv2
from dataclasses import dataclass, field
from typing import Dict, List, Optional


@dataclass
class SourceMedia:
    id: int
    path: str
    fps: float
    duration: float
    width: int
    height: int


def probe_media(path: str) -> SourceMedia:
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open media: {path}")
    fps = float(cap.get(cv2.CAP_PROP_FPS) or 30.0)
    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
    duration = float(frame_count) / float(fps) if fps > 0 and frame_count > 0 else 0.0
    cap.release()
    return SourceMedia(id=-1, path=path, fps=fps, duration=duration, width=width, height=height)


@dataclass
class Clip:
    id: int
    source_id: int
    # in source time
    source_in: float
    source_out: float
    # on timeline
    timeline_start: float
    track_index: int

    
    @property
    def timeline_end(self) -> float:
        return self.timeline_start + max(0.0, self.source_out - self.source_in)


@dataclass
class Project:
    sources: Dict[int, SourceMedia] = field(default_factory=dict)
    clips: Dict[int, Clip] = field(default_factory=dict)
    next_source_id: int = 1
    next_clip_id: int = 1
    track_count: int = 3

    
    def add_source(self, path: str) -> SourceMedia:
        media = probe_media(path)
        media.id = self.next_source_id
        self.sources[media.id] = media
        self.next_source_id += 1
        return media

    
    def add_clip(self, source_id: int, source_in: float, source_out: float, timeline_start: float, track_index: int) -> Clip:
        clip = Clip(
            id=self.next_clip_id,
            source_id=source_id,
            source_in=max(0.0, min(source_in, source_out)),
            source_out=max(source_in, source_out),
            timeline_start=max(0.0, timeline_start),
            track_index=max(0, track_index),
        )
        self.clips[clip.id] = clip
        self.next_clip_id += 1
        return clip

    
    def remove_clip(self, clip_id: int) -> None:
        if clip_id in self.clips:
            del self.clips[clip_id]
    
    
    def get_timeline_duration(self) -> float:
        if not self.clips:
            return 0.0
        return max(c.timeline_end for c in self.clips.values())


