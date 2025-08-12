from __future__ import annotations

import sys
from argparse import ArgumentParser

from .browser import Browser
from .renderer import render_for_terminal, truncate_text, render_html_to_terminal
from .logging_utils import get_app_logger


def main(argv: list[str] | None = None) -> int:
    parser = ArgumentParser(prog="my-own-browser", description="Tiny terminal browser")
    parser.add_argument("url", help="URL to open (http(s):// or domain)")
    parser.add_argument("--width", type=int, default=100, help="Max terminal width for wrapping")
    parser.add_argument("--max-chars", type=int, default=2000, help="Maximum characters to render")
    parser.add_argument("--pretty", action="store_true", help="Use structured HTML rendering")

    args = parser.parse_args(argv)

    logger = get_app_logger("my_own_browser.cli")
    logger.info("CLI start")
    browser = Browser()
    page = browser.go(args.url)
    logger.info("Fetched %s (%s)", page.url, page.status_code)

    header_lines = [
        f"Title: {page.title or '(no title)'}",
        f"URL:   {page.url}",
        f"HTTP:  {page.status_code}",
        "",
    ]

    if args.pretty:
        body = render_html_to_terminal(page.content, base_url=page.url, max_width=args.width, max_chars=args.max_chars)
        rendered = "\n".join(header_lines) + "\n" + body
    else:
        body = truncate_text(page.text, args.max_chars)
        rendered = render_for_terminal(header_lines + [body], max_width=args.width)
    print(rendered)
    if page.links:
        print("\nLinks:")
        for idx, link in enumerate(page.links[:20], start=1):
            print(f"  [{idx}] {link}")
    logger.info("Done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))


