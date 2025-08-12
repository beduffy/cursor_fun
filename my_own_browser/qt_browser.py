from __future__ import annotations

"""A minimal Qt-based GUI browser using QtWebEngine.

This uses the system's Qt WebEngine (Chromium) for full HTML/CSS/JS rendering.
It's a pragmatic way to get a visual browser while we iterate on our own core.

Run:
    python -m my_own_browser.qt_browser

Requires:
    pip install PySide6
"""

import sys
from typing import Optional


def guess_url(text: str) -> str:
    text = (text or "").strip()
    if not text:
        return "about:blank"
    if "://" in text or text.startswith("about:"):
        return text
    # Treat as domain or hostname; default to https
    return f"https://{text}"


def main(argv: Optional[list[str]] = None) -> int:
    try:
        from PySide6.QtCore import QUrl
        from PySide6.QtWidgets import (
            QApplication,
            QHBoxLayout,
            QLineEdit,
            QMainWindow,
            QPushButton,
            QStatusBar,
            QToolBar,
            QVBoxLayout,
            QWidget,
        )
        from PySide6.QtWebEngineWidgets import QWebEngineView
    except Exception as exc:  # pragma: no cover - exercised at runtime
        sys.stderr.write(
            "PySide6 with QtWebEngine is required.\n"
            "Install with: pip install PySide6\n"
            f"Error: {exc}\n"
        )
        return 1

    app = QApplication(sys.argv)

    class BrowserWindow(QMainWindow):
        def __init__(self) -> None:
            super().__init__()
            self.setWindowTitle("My Own Browser")
            self.resize(1200, 800)

            # Toolbar
            toolbar = QToolBar("Navigation")
            self.addToolBar(toolbar)

            self.back_button = QPushButton("←")
            self.forward_button = QPushButton("→")
            self.reload_button = QPushButton("⟳")
            self.url_input = QLineEdit()
            self.url_input.setPlaceholderText("Enter URL…")

            toolbar_layout = QHBoxLayout()
            toolbar_widget = QWidget()
            toolbar_widget.setLayout(toolbar_layout)
            toolbar.addWidget(toolbar_widget)

            toolbar_layout.addWidget(self.back_button)
            toolbar_layout.addWidget(self.forward_button)
            toolbar_layout.addWidget(self.reload_button)
            toolbar_layout.addWidget(self.url_input, stretch=1)

            # Web view
            self.web = QWebEngineView()

            # Central layout
            central = QWidget()
            vbox = QVBoxLayout(central)
            vbox.setContentsMargins(0, 0, 0, 0)
            vbox.addWidget(self.web)
            self.setCentralWidget(central)

            # Status bar
            self.status = QStatusBar()
            self.setStatusBar(self.status)

            # Signals
            self.back_button.clicked.connect(self.web.back)
            self.forward_button.clicked.connect(self.web.forward)
            self.reload_button.clicked.connect(self.web.reload)
            self.url_input.returnPressed.connect(self._on_url_entered)
            self.web.urlChanged.connect(self._on_url_changed)
            self.web.loadProgress.connect(self._on_progress)

        def _on_url_entered(self) -> None:
            target = guess_url(self.url_input.text())
            self.web.setUrl(QUrl(target))

        def _on_url_changed(self, url: QUrl) -> None:  # type: ignore[name-defined]
            self.url_input.setText(url.toString())

        def _on_progress(self, val: int) -> None:
            self.status.showMessage(f"Loading… {val}%")
            if val == 100:
                self.status.clearMessage()

    win = BrowserWindow()
    win.show()
    return app.exec()


if __name__ == "__main__":  # pragma: no cover - manual run
    raise SystemExit(main(sys.argv[1:]))


