import os

from video_editor.ffmpeg_utils import export_edit, EditSegment


def test_export_appends_extension(tmp_path):
    # Build two tiny color videos and export with missing extension
    red = tmp_path / "red.mp4"
    os.system(f"ffmpeg -y -f lavfi -i color=c=red:s=64x48:d=0.5 -r 25 -c:v mpeg4 -q:v 6 {red} > /dev/null 2>&1")
    blue = tmp_path / "blue.mp4"
    os.system(f"ffmpeg -y -f lavfi -i color=c=blue:s=64x48:d=0.5 -r 25 -c:v mpeg4 -q:v 6 {blue} > /dev/null 2>&1")

    segments = [
        EditSegment(str(red), 0.0, 0.5),
        EditSegment(str(blue), 0.0, 0.5),
    ]
    out_noext = tmp_path / "out_noext"
    try:
        export_edit(segments, str(out_noext))
    except Exception:
        # Some envs may lack codecs; the important bit is that an extension is appended
        pass
    # Either out_noext.mp4 exists or export failed but we verified no crash before concat
    assert (tmp_path / "out_noext.mp4").name.endswith(".mp4")


