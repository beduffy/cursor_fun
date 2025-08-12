from __future__ import annotations

import json
import os

from my_own_browser.storage import load_settings, save_settings, get_settings_path, get_app_paths


def test_settings_round_trip(tmp_path, monkeypatch):
    fake_home = tmp_path / "home"
    fake_home.mkdir()
    monkeypatch.setenv("HOME", str(fake_home))

    # Initially empty
    assert load_settings("test_browser") == {}

    data = {"homepage": "https://example.com", "bookmarks": ["https://ex.com"]}
    save_settings(data, "test_browser")

    loaded = load_settings("test_browser")
    assert loaded == data

    # Path exists
    path = get_settings_path("test_browser")
    assert os.path.isfile(path)
    on_disk = json.loads(open(path, "r", encoding="utf-8").read())
    assert on_disk == data


