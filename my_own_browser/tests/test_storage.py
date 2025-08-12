from __future__ import annotations

import os

from my_own_browser.storage import get_app_paths


def test_get_app_paths_creates_dirs(tmp_path, monkeypatch):
    fake_home = tmp_path / "home"
    fake_home.mkdir()
    monkeypatch.setenv("HOME", str(fake_home))

    paths = get_app_paths("test_app")
    assert os.path.isdir(paths.config_dir)
    assert os.path.isdir(paths.cache_dir)
    assert os.path.isdir(paths.downloads_dir)



