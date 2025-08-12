## Video Editor

A minimal desktop video editor built with PySide6 and ffmpeg.

MVP list editor:
- Load videos and preview them (OpenCV-based preview)
- Play/pause and scrub with a timeline
- Mark In/Out points and add segments to an edit list
- Reorder or delete segments
- Export via ffmpeg (re-encodes for safety)

Multi-track timeline (DaVinci-style basics):
- Media bin with multiple sources (supports OS drag-and-drop)
- Timeline with multiple tracks; drag to move, edge-trim; vertical drag to change track
- Selection: single, Ctrl multi-select, marquee (Shift+drag)
- Snapping to playhead and clip edges (G to toggle)
- Split at playhead (S), Delete selected, Duplicate (D)
- Zoom (Ctrl+Wheel, +/− buttons), Pan (Shift+Wheel), horizontal scrollbar
- Follow playhead (F)
- Track header: lock/mute/visibility (per-track state persisted)
- Export flattens top-most visible track priority; encoder selection is probed; appends .mp4 extension when omitted

### Quickstart

1) Ensure `ffmpeg` is installed and available on your PATH.
   - Linux (Debian/Ubuntu): `sudo apt-get update && sudo apt-get install -y ffmpeg`
   - Verify: `ffmpeg -version`

2) (Optional, recommended) Create and activate a conda env:
   ```bash
   conda create -y -n video_editor_env python=3.11
   conda activate video_editor_env
   ```

3) Install Python dependencies:
   ```bash
   pip install -r requirements.txt
   ```

4) Run the simple editor (In/Out list):
   ```bash
   python app.py
   ```

5) Run the multi-track timeline editor:
```bash
python app_timeline.py
```

### Usage

- File → Open Video… to load sources, or drag files into the media bin
- Space: Play/Pause
- I / O / A: In, Out, Add Segment (list editor)
- S: Split at playhead (timeline)
- D: Duplicate selected clip
- Delete: Remove selected clips
- G: Toggle snapping
- F: Toggle follow-playhead
- Ctrl+Wheel / + / −: Zoom; Shift+Wheel: Pan; horizontal slider: pan
- Marquee selection: Shift+drag on empty timeline
- Track header click: toggle lock; context toggles via shortcuts (M/V in app)
- Export: Choose output path and render via ffmpeg

Notes:
- Export uses re-encoding (prefers `libx264` + `aac` if present; falls back to available encoders).
- Preview uses OpenCV for frames; audio is not played yet.

### Tests

Run unit and integration tests (ffmpeg must be installed):
```bash
PYTHONPATH=. pytest -q video_editor/tests
```

### Roadmap

- Ripple/roll/slip/slide trims; visual snap guides
- Audio waveform and audio preview
- Basic transitions (crossfade), text overlays, and filters
- Per-clip scaling/cropping and speed adjustments
- Project autosave, relink missing media, undo coalescing


