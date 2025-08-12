from __future__ import annotations

"""Render Mermaid diagrams to PNG using Playwright.

Usage:
  python -m my_own_browser.render_mermaid --in flow.mmd --out flow.png

Requires:
  pip install playwright
  playwright install chromium
"""

import argparse
import base64
import sys
from typing import Optional


HTML_WRAPPER = """
<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <script type="module">
    import mermaid from 'https://cdn.jsdelivr.net/npm/mermaid@10/dist/mermaid.esm.min.mjs';
    mermaid.initialize({ startOnLoad: true, theme: 'default' });
    window.renderDiagram = async (code) => {
      const el = document.getElementById('d');
      const { svg } = await mermaid.render('graph1', code);
      el.innerHTML = svg;
      return true;
    };
  </script>
  <style>body,html{margin:0;padding:0}</style>
  <title>Mermaid Render</title>
  </head>
  <body>
    <div id="d"></div>
  </body>
</html>
"""


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Render Mermaid to PNG")
    p.add_argument("--in", dest="infile", required=True, help="Input .mmd file")
    p.add_argument("--out", dest="outfile", required=True, help="Output .png path")
    p.add_argument("--width", type=int, default=1200)
    p.add_argument("--height", type=int, default=800)
    return p.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)
    code = open(args.infile, "r", encoding="utf-8").read()

    try:
        from playwright.sync_api import sync_playwright
    except Exception as exc:  # pragma: no cover
        sys.stderr.write(
            "Playwright is required.\n"
            "Install with: pip install playwright && playwright install chromium\n"
            f"Error: {exc}\n"
        )
        return 1

    with sync_playwright() as p:
        browser = p.chromium.launch()
        context = browser.new_context(viewport={"width": args.width, "height": args.height})
        page = context.new_page()
        page.set_content(HTML_WRAPPER)
        page.wait_for_load_state("domcontentloaded")
        page.evaluate("window.renderDiagram", code)
        page.wait_for_timeout(300)  # allow rendering
        el = page.locator("#d svg")
        el.wait_for()
        # Use element screenshot to fit size
        png = el.screenshot(type="png")
        open(args.outfile, "wb").write(png)
        browser.close()
    print(f"Wrote {args.outfile}")
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main(sys.argv[1:]))


