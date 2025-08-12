from __future__ import annotations

from my_own_browser.history import add_to_history, load_history


def test_history_add_and_load(tmp_path, monkeypatch):
    fake_home = tmp_path / "home"
    fake_home.mkdir()
    monkeypatch.setenv("HOME", str(fake_home))

    add_to_history("https://a.com", "A", app_name="test_hist")
    add_to_history("https://b.com", "B", app_name="test_hist")
    items = load_history(app_name="test_hist")
    assert [i.url for i in items] == ["https://a.com", "https://b.com"]


