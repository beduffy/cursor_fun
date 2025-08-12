from __future__ import annotations

from my_own_browser.screenshot import parse_args


def test_parse_args():
    args = parse_args(["--url", "https://example.com", "--out", "out.png", "--width", "800", "--height", "600", "--full-page"])
    assert args.url == "https://example.com"
    assert args.out == "out.png"
    assert args.width == 800
    assert args.height == 600
    assert args.full_page is True


