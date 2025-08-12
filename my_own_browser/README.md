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

Headless screenshot (proves real rendering):

```bash
pip install playwright
playwright install chromium
python3 -m my_own_browser.screenshot --url https://example.com --out example_screenshot.png --full-page
```

## Tests

```bash
pytest -q my_own_browser/tests
```


