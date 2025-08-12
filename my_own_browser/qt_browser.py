from __future__ import annotations

"""A minimal Qt-based GUI browser using QtWebEngine.

This uses the system's Qt WebEngine (Chromium) for full HTML/CSS/JS rendering.
It's a pragmatic way to get a visual browser while we iterate on our own core.

Run:
    python -m my_own_browser.qt_browser

Requires:
    pip install PySide6
"""

import os
import sys
from typing import Optional

from .storage import get_app_paths
from .bookmarks import add_bookmark, load_bookmarks
from .history import add_to_history


def guess_url(text: str) -> str:
    text = (text or "").strip()
    if not text:
        return "about:blank"
    if "://" in text or text.startswith("about:"):
        return text
    # Treat as domain or hostname; default to https
    return f"https://{text}"


def main(argv: Optional[list[str]] = None) -> int:
    # Prefer X11 (xcb) by default to avoid Wayland/libffi issues in some envs
    os.environ.setdefault("QT_QPA_PLATFORM", "xcb")
    try:
        from PySide6.QtCore import QUrl
        from PySide6.QtWidgets import (
            QApplication,
            QHBoxLayout,
            QLineEdit,
            QMainWindow,
            QPushButton,
            QTabWidget,
            QListWidget,
            QDialog,
            QDialogButtonBox,
            QStatusBar,
            QToolBar,
            QVBoxLayout,
            QWidget,
        )
        from PySide6.QtGui import QShortcut, QKeySequence
        from PySide6.QtWebEngineCore import (
            QWebEngineProfile,
            QWebEnginePage,
            QWebEngineDownloadRequest,
        )
        from PySide6.QtWebEngineWidgets import QWebEngineView
    except Exception as exc:  # pragma: no cover - exercised at runtime
        sys.stderr.write(
            "PySide6 with QtWebEngine is required.\n"
            "Try: pip install PySide6 (or conda install -c conda-forge pyside6)\n"
            "If you see Wayland/libffi errors, try: QT_QPA_PLATFORM=xcb python -m my_own_browser.qt_browser\n"
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
            self.bookmark_button = QPushButton("★")
            self.url_input = QLineEdit()
            self.url_input.setPlaceholderText("Enter URL…")

            toolbar_layout = QHBoxLayout()
            toolbar_widget = QWidget()
            toolbar_widget.setLayout(toolbar_layout)
            toolbar.addWidget(toolbar_widget)

            toolbar_layout.addWidget(self.back_button)
            toolbar_layout.addWidget(self.forward_button)
            toolbar_layout.addWidget(self.reload_button)
            toolbar_layout.addWidget(self.bookmark_button)
            toolbar_layout.addWidget(self.url_input, stretch=1)

            # Persistence paths and profile
            self.paths = get_app_paths()
            self.profile = QWebEngineProfile("Default", self)
            try:
                self.profile.setCachePath(self.paths.cache_dir)
                self.profile.setPersistentStoragePath(self.paths.config_dir)
                if hasattr(self.profile, "setPersistentCookiesPolicy"):
                    self.profile.setPersistentCookiesPolicy(QWebEngineProfile.ForcePersistentCookies)  # type: ignore[attr-defined]
            except Exception:
                pass

            # Downloads
            try:
                self.profile.downloadRequested.connect(self._on_download_requested)  # type: ignore[attr-defined]
            except Exception:
                pass

            # Tabs
            self.tabs = QTabWidget()
            self.tabs.setTabsClosable(True)
            self.tabs.tabCloseRequested.connect(self._close_tab)
            self.tabs.currentChanged.connect(self._on_tab_changed)

            # Create initial tab
            self.homepage = "https://example.com"
            self._add_tab(self.homepage)

            # Central layout
            central = QWidget()
            vbox = QVBoxLayout(central)
            vbox.setContentsMargins(0, 0, 0, 0)
            vbox.addWidget(self.tabs)
            self.setCentralWidget(central)

            # Status bar
            self.status = QStatusBar()
            self.setStatusBar(self.status)
            self.status.setSizeGripEnabled(True)

            # Signals
            self.back_button.clicked.connect(lambda: self._current_web().back())
            self.forward_button.clicked.connect(lambda: self._current_web().forward())
            self.reload_button.clicked.connect(lambda: self._current_web().reload())
            self.url_input.returnPressed.connect(self._on_url_entered)
            self.bookmark_button.clicked.connect(self._on_add_bookmark)

            # Connect for current tab
            self._connect_current_tab()

            # Shortcuts
            QShortcut(QKeySequence("Ctrl+L"), self, activated=self._focus_url)
            QShortcut(QKeySequence("Ctrl+T"), self, activated=lambda: self._add_tab(self.homepage))
            QShortcut(QKeySequence("Ctrl+W"), self, activated=lambda: self._close_tab(self.tabs.currentIndex()))
            QShortcut(QKeySequence("Ctrl+R"), self, activated=lambda: self._current_web().reload())
            QShortcut(QKeySequence("Ctrl+B"), self, activated=self._show_bookmarks)
            QShortcut(QKeySequence("Ctrl+H"), self, activated=self._show_history)
            QShortcut(QKeySequence("Ctrl+F"), self, activated=self._find_in_page)

        def _current_web(self) -> QWebEngineView:  # type: ignore[name-defined]
            return self.tabs.currentWidget()  # type: ignore[return-value]

        def _add_tab(self, target: str) -> None:
            web = BrowserTabView(self)
            web.setPage(QWebEnginePage(self.profile, web))
            web.urlChanged.connect(self._on_url_changed)
            web.loadProgress.connect(self._on_progress)
            self.tabs.addTab(web, "New Tab")
            web.setUrl(QUrl(target))

        def _close_tab(self, index: int) -> None:
            if self.tabs.count() > 1:
                self.tabs.removeTab(index)

        def _on_tab_changed(self, index: int) -> None:
            self._connect_current_tab()
            cur = self._current_web()
            if cur:
                self.url_input.setText(cur.url().toString())

        def _connect_current_tab(self) -> None:
            # Tabs already connect signals on creation; nothing else needed now.
            pass

        def _on_url_entered(self) -> None:
            target = guess_url(self.url_input.text())
            cur = self._current_web()
            cur.setUrl(QUrl(target))

        def _on_url_changed(self, url: QUrl) -> None:  # type: ignore[name-defined]
            self.url_input.setText(url.toString())
            # Update tab title
            cur = self._current_web()
            if cur:
                self.tabs.setTabText(self.tabs.currentIndex(), url.toString()[:30])
                # Add to history
                try:
                    add_to_history(url.toString(), url.toString())
                except Exception:
                    pass

        def _on_progress(self, val: int) -> None:
            self.status.showMessage(f"Loading… {val}%")
            if val == 100:
                self.status.clearMessage()

        def _on_download_requested(self, req: QWebEngineDownloadRequest) -> None:  # type: ignore[name-defined]
            try:
                filename = req.downloadFileName()
                req.setDownloadDirectory(self.paths.downloads_dir)
                req.setDownloadFileName(filename)
                req.accept()
                save_path = self.paths.downloads_dir + "/" + filename
                self.status.showMessage(f"Downloading to {save_path}")
                req.finished.connect(lambda: self.status.showMessage(f"Downloaded: {save_path}", 5000))
            except Exception as exc:
                self.status.showMessage(f"Download failed: {exc}", 5000)

        def _focus_url(self) -> None:
            self.url_input.setFocus()

        def _on_add_bookmark(self) -> None:
            cur = self._current_web()
            if not cur:
                return
            url = cur.url().toString()
            title = url
            try:
                add_bookmark(url, title)
                self.status.showMessage("Bookmarked", 2000)
            except Exception as exc:
                self.status.showMessage(f"Bookmark failed: {exc}", 3000)

        def _show_bookmarks(self) -> None:
            items = load_bookmarks()
            dlg = QDialog(self)
            dlg.setWindowTitle("Bookmarks")
            v = QVBoxLayout(dlg)
            listw = QListWidget(dlg)
            for it in items:
                listw.addItem(f"{it.title} — {it.url}")
            v.addWidget(listw)
            buttons = QDialogButtonBox(QDialogButtonBox.Close)
            buttons.rejected.connect(dlg.reject)
            v.addWidget(buttons)
            listw.itemDoubleClicked.connect(lambda item: (self._navigate_to(item.text().split(" — ")[-1]), dlg.accept()))
            dlg.exec()

        def _navigate_to(self, url: str) -> None:
            cur = self._current_web()
            if cur:
                from PySide6.QtCore import QUrl as _QUrl
                cur.setUrl(_QUrl(url))

        def _show_history(self) -> None:
            from .history import load_history
            items = load_history()
            dlg = QDialog(self)
            dlg.setWindowTitle("History")
            v = QVBoxLayout(dlg)
            listw = QListWidget(dlg)
            for it in items[-500:]:
                listw.addItem(f"{it.title} — {it.url}")
            v.addWidget(listw)
            buttons = QDialogButtonBox(QDialogButtonBox.Close)
            buttons.rejected.connect(dlg.reject)
            v.addWidget(buttons)
            listw.itemDoubleClicked.connect(lambda item: (self._navigate_to(item.text().split(" — ")[-1]), dlg.accept()))
            dlg.exec()

        def _find_in_page(self) -> None:
            cur = self._current_web()
            if not cur:
                return
            from PySide6.QtWidgets import QInputDialog
            text, ok = QInputDialog.getText(self, "Find", "Find text:")
            if ok and text:
                try:
                    cur.findText(text)
                except Exception:
                    pass

    class BrowserTabView(QWebEngineView):  # type: ignore[misc]
        def __init__(self, window: BrowserWindow) -> None:
            super().__init__(window)
            self.window = window

        def createWindow(self, _type):  # noqa: N802 - Qt overrides camelCase
            # Open target=_blank and window.open in a new tab
            self.window._add_tab("about:blank")
            return self.window._current_web()

    win = BrowserWindow()
    win.show()
    return app.exec()


if __name__ == "__main__":  # pragma: no cover - manual run
    raise SystemExit(main(sys.argv[1:]))


