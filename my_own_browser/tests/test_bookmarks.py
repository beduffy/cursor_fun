from __future__ import annotations

import os

from my_own_browser.bookmarks import add_bookmark, load_bookmarks


def test_add_and_load_bookmarks(tmp_path, monkeypatch):
    fake_home = tmp_path / "home"
    fake_home.mkdir()
    monkeypatch.setenv("HOME", str(fake_home))

    bm = add_bookmark("https://example.com", "Example", app_name="test_bm")
    assert bm.url == "https://example.com"
    assert bm.title == "Example"

    items = load_bookmarks(app_name="test_bm")
    assert len(items) == 1
    assert items[0].url == "https://example.com"


