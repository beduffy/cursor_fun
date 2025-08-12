from __future__ import annotations

"""Headless screenshot tool using Playwright.

Usage:
    python -m my_own_browser.screenshot --url https://example.com --out example.png

Requires:
    pip install playwright
    python -m playwright install chromium
"""

import argparse
import sys
from typing import Optional, List
from .logging_utils import get_app_logger


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Headless page screenshot (Chromium)")
    parser.add_argument("--url", required=True, help="URL to screenshot")
    parser.add_argument("--out", required=True, help="Output PNG path")
    parser.add_argument("--width", type=int, default=1280, help="Viewport width")
    parser.add_argument("--height", type=int, default=800, help="Viewport height")
    parser.add_argument("--full-page", action="store_true", help="Capture full page height")
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    logger = get_app_logger("my_own_browser.screenshot")
    logger.info("Screenshot start: %s -> %s", args.url, args.out)
    try:
        from playwright.sync_api import sync_playwright
    except Exception as exc:  # pragma: no cover
        sys.stderr.write(
            "Playwright is required.\n"
            "Install with: pip install playwright\n"
            "Then install a browser: python -m playwright install chromium\n"
            f"Error: {exc}\n"
        )
        return 1

    with sync_playwright() as p:
        browser = p.chromium.launch(headless=True)
        try:
            context = browser.new_context(viewport={"width": args.width, "height": args.height})
            page = context.new_page()
            page.goto(args.url, wait_until="domcontentloaded")
            page.screenshot(path=args.out, full_page=args.full_page)
        finally:
            browser.close()
    print(f"Saved screenshot to {args.out}")
    logger.info("Saved screenshot to %s", args.out)
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main(sys.argv[1:]))


