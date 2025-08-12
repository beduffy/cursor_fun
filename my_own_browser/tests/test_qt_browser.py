from __future__ import annotations

from my_own_browser.qt_browser import guess_url


def test_guess_url():
    assert guess_url("") == "about:blank"
    assert guess_url("example.com") == "https://example.com"
    assert guess_url("http://example.com") == "http://example.com"
    assert guess_url("https://example.com") == "https://example.com"
    assert guess_url(" about:blank ") == "about:blank"


