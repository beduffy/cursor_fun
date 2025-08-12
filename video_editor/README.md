## Video Editor (MVP)

A minimal desktop video editor built with PySide6 and ffmpeg. It lets you:

- Load videos and preview them
- Play/pause and scrub with a timeline
- Mark In/Out points and add segments to an edit list
- Reorder or delete segments
- Export the final edit via ffmpeg (re-encodes for safety)

New multi-track MVP (DaVinci-style basics):

- Media bin, add multiple sources
- Timeline with multiple tracks
- Drag to move clips horizontally, trim at edges
- Export flattens the timeline by top-most clip priority

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

- File → Open Video… to load a source video. You can load multiple sources; pick the active one from the dropdown at the top.
- Space: Play/Pause
- I: Set In point at current time
- O: Set Out point at current time
- A: Add current [In, Out] segment to the edit list
- Delete: Remove selected segment(s) from the edit list
- Drag to reorder segments in the list
- Export: Choose output path and render the concatenated edit via ffmpeg

Notes:
- Export uses re-encoding (`libx264` + `aac`) for compatibility and consistent output.
- Preview uses OpenCV; audio is not played in the preview for simplicity.

### Tests

Run unit and integration tests (ffmpeg must be installed):
```bash
pytest -q
```

### Roadmap

- Ripple trim, split on playhead
- Audio waveform and audio preview
- Basic transitions (crossfade), text overlays, and filters
- Per-clip scaling/cropping and speed adjustments
- Project save/load (.json)


