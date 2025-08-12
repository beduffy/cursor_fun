from __future__ import annotations

from my_own_browser.browser import Browser


def test_normalize_url_adds_scheme():
    b = Browser()
    assert b._normalize_url("example.com") == "https://example.com"
    assert b._normalize_url("http://example.com") == "http://example.com"
    assert b._normalize_url("https://example.com") == "https://example.com"


