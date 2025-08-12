from __future__ import annotations

import cv2
from dataclasses import dataclass, field
from typing import Dict, List, Optional
import json
import os


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
    track_locks: List[bool] = field(default_factory=lambda: [False, False, False])
    track_mute: List[bool] = field(default_factory=lambda: [False, False, False])
    track_visible: List[bool] = field(default_factory=lambda: [True, True, True])

    
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
    
    
    def remove_clips(self, clip_ids: List[int]) -> None:
        for clip_id in clip_ids:
            if clip_id in self.clips:
                del self.clips[clip_id]


    def get_timeline_duration(self) -> float:
        if not self.clips:
            return 0.0
        return max(c.timeline_end for c in self.clips.values())

    
    def split_clip(self, clip_id: int, split_time: float) -> Optional[Clip]:
        """Split a clip at absolute timeline time.

        Returns the new right-hand clip if split occurred, else None.
        """
        clip = self.clips.get(clip_id)
        if clip is None:
            return None
        if not (clip.timeline_start < split_time < clip.timeline_end):
            return None
        left_duration = max(0.0, split_time - clip.timeline_start)
        right_duration = max(0.0, clip.timeline_end - split_time)
        # Adjust left
        clip.source_out = clip.source_in + left_duration
        # Create right
        right_clip = Clip(
            id=self.next_clip_id,
            source_id=clip.source_id,
            source_in=clip.source_out,
            source_out=clip.source_out + right_duration,
            timeline_start=split_time,
            track_index=clip.track_index,
        )
        self.clips[right_clip.id] = right_clip
        self.next_clip_id += 1
        return right_clip

    
    def duplicate_clip(self, clip_id: int) -> Optional[Clip]:
        clip = self.clips.get(clip_id)
        if clip is None:
            return None
        dup = Clip(
            id=self.next_clip_id,
            source_id=clip.source_id,
            source_in=clip.source_in,
            source_out=clip.source_out,
            timeline_start=clip.timeline_start,
            track_index=clip.track_index,
        )
        self.clips[dup.id] = dup
        self.next_clip_id += 1
        return dup

    
    def to_dict(self) -> dict:
        return {
            "sources": [
                {
                    "id": s.id,
                    "path": s.path,
                    "fps": s.fps,
                    "duration": s.duration,
                    "width": s.width,
                    "height": s.height,
                }
                for s in self.sources.values()
            ],
            "clips": [
                {
                    "id": c.id,
                    "source_id": c.source_id,
                    "source_in": c.source_in,
                    "source_out": c.source_out,
                    "timeline_start": c.timeline_start,
                    "track_index": c.track_index,
                }
                for c in self.clips.values()
            ],
            "next_source_id": self.next_source_id,
            "next_clip_id": self.next_clip_id,
            "track_count": self.track_count,
            "track_locks": list(self.track_locks) if self.track_locks else [False] * self.track_count,
            "track_mute": list(self.track_mute) if self.track_mute else [False] * self.track_count,
            "track_visible": list(self.track_visible) if self.track_visible else [True] * self.track_count,
        }

    
    @classmethod
    def from_dict(cls, data: dict) -> "Project":
        proj = cls()
        proj.next_source_id = data.get("next_source_id", 1)
        proj.next_clip_id = data.get("next_clip_id", 1)
        proj.track_count = data.get("track_count", 3)
        proj.track_locks = data.get("track_locks", [False] * proj.track_count)
        proj.track_mute = data.get("track_mute", [False] * proj.track_count)
        proj.track_visible = data.get("track_visible", [True] * proj.track_count)
        for s in data.get("sources", []):
            proj.sources[s["id"]] = SourceMedia(
                id=s["id"],
                path=s["path"],
                fps=s.get("fps", 30.0),
                duration=s.get("duration", 0.0),
                width=s.get("width", 0),
                height=s.get("height", 0),
            )
        for c in data.get("clips", []):
            proj.clips[c["id"]] = Clip(
                id=c["id"],
                source_id=c["source_id"],
                source_in=c["source_in"],
                source_out=c["source_out"],
                timeline_start=c["timeline_start"],
                track_index=c.get("track_index", 0),
            )
        return proj

    
    def save_project(self, path: str) -> None:
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self.to_dict(), f, indent=2)

    
    @classmethod
    def load_project(cls, path: str) -> "Project":
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        return cls.from_dict(data)

    
    def ensure_track_arrays(self) -> None:
        if len(self.track_locks) < self.track_count:
            self.track_locks.extend([False] * (self.track_count - len(self.track_locks)))
        if len(self.track_mute) < self.track_count:
            self.track_mute.extend([False] * (self.track_count - len(self.track_mute)))
        if len(self.track_visible) < self.track_count:
            self.track_visible.extend([True] * (self.track_count - len(self.track_visible)))

    
    def set_track_count(self, count: int) -> None:
        self.track_count = max(1, count)
        self.ensure_track_arrays()

    
    def is_track_locked(self, track_index: int) -> bool:
        self.ensure_track_arrays()
        if 0 <= track_index < len(self.track_locks):
            return bool(self.track_locks[track_index])
        return False

    
    def toggle_track_lock(self, track_index: int) -> None:
        self.ensure_track_arrays()
        if 0 <= track_index < len(self.track_locks):
            self.track_locks[track_index] = not self.track_locks[track_index]

    
    def toggle_track_mute(self, track_index: int) -> None:
        self.ensure_track_arrays()
        if 0 <= track_index < len(self.track_mute):
            self.track_mute[track_index] = not self.track_mute[track_index]

    
    def toggle_track_visible(self, track_index: int) -> None:
        self.ensure_track_arrays()
        if 0 <= track_index < len(self.track_visible):
            self.track_visible[track_index] = not self.track_visible[track_index]


@dataclass
class ProjectHistory:
    snapshots: List[dict] = field(default_factory=list)
    index: int = -1

    
    def push(self, project: Project) -> None:
        # Drop redo tail
        if self.index < len(self.snapshots) - 1:
            self.snapshots = self.snapshots[: self.index + 1]
        self.snapshots.append(project.to_dict())
        self.index = len(self.snapshots) - 1

    
    def can_undo(self) -> bool:
        return self.index > 0

    
    def can_redo(self) -> bool:
        return self.index < len(self.snapshots) - 1

    
    def undo(self, project: Project) -> bool:
        if not self.can_undo():
            return False
        self.index -= 1
        data = self.snapshots[self.index]
        restored = Project.from_dict(data)
        project.sources = restored.sources
        project.clips = restored.clips
        project.next_source_id = restored.next_source_id
        project.next_clip_id = restored.next_clip_id
        project.track_count = restored.track_count
        project.track_locks = restored.track_locks
        return True

    
    def redo(self, project: Project) -> bool:
        if not self.can_redo():
            return False
        self.index += 1
        data = self.snapshots[self.index]
        restored = Project.from_dict(data)
        project.sources = restored.sources
        project.clips = restored.clips
        project.next_source_id = restored.next_source_id
        project.next_clip_id = restored.next_clip_id
        project.track_count = restored.track_count
        project.track_locks = restored.track_locks
        return True


