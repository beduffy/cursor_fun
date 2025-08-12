# My Own Browser

This package contains a tiny, testable, iterative toy browser.

## Quick start

```bash
python -m my_own_browser.cli https://example.com
```

GUI (requires PySide6):

```bash
# Install GUI dep (recommend conda-forge on Linux)
pip install PySide6

# Launch (Wayland-safe)
QT_QPA_PLATFORM=xcb python -m my_own_browser.qt_browser https://example.com

# Shortcuts: Ctrl+L (URL), Ctrl+T/W (new/close tab), Ctrl+R (reload), Ctrl+B (bookmarks), Ctrl+H (history), Ctrl+F (find)
```

Features:
- Tabs, persistent cookies/cache, downloads to `~/Downloads`
- Bookmarks (★ button, Ctrl+B) saved under `~/.config/my_own_browser/bookmarks.json`
- History (Ctrl+H) saved under `~/.config/my_own_browser/history.json`
- Find in page (Ctrl+F)
- Homepage (Alt+Home), set current as homepage (Ctrl+Shift+H)
- Reader mode (Ctrl+Shift+R) renders current page in a side dock using the built-in text renderer
- Basic URL blocklist (ad/tracker patterns) editable at `~/.config/my_own_browser/blocklist.json`
- Zoom controls: Ctrl++ / Ctrl+- / Ctrl+0 and +/- buttons; persisted between runs
- URL autocompletion from bookmarks and history

Headless screenshot (proves real rendering):

```bash
pip install playwright
playwright install chromium
python3 -m my_own_browser.screenshot --url https://example.com --out example_screenshot.png --full-page
```

Logs:

```text
~/.config/my_own_browser/logs/app.log
```

## Tests

```bash
pytest -q my_own_browser/tests
```


