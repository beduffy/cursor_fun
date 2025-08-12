from __future__ import annotations

from my_own_browser.filter import Blocklist


def test_blocklist_matches():
    bl = Blocklist(patterns=["ads", "track", "metrics.example.com"])
    assert bl.matches("https://foo.com/ads/banner.js")
    assert bl.matches("https://metrics.example.com/pixel")
    assert not bl.matches("https://example.com/content")


