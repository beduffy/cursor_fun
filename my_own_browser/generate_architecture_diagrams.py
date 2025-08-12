from __future__ import annotations

import os

ARCH = """
flowchart TD
  User["User Actions\\n(URL bar, buttons, shortcuts)"] --> GUI[(Qt GUI)]
  GUI -->|navigate| Fetcher[Browser.fetch]
  Fetcher --> HTML[(HTML)]
  HTML -->|rendered-by| QtWebEngine[(Chromium engine)]
  HTML --> Reader[Text Renderer]
  Reader --> GUI
  GUI --> Persist[Bookmarks/History/Settings]
  Persist --> Files[(~/.config/my_own_browser/...)]
  GUI --> Downloads[(~/Downloads)]
  GUI --> Logs[(~/.config/my_own_browser/logs/app.log)]
"""

READER = """
sequenceDiagram
  participant GUI
  participant Requests
  participant Renderer
  GUI->>Requests: GET URL (reader mode)
  Requests-->>GUI: HTML
  GUI->>Renderer: render_html_to_terminal(html, base_url)
  Renderer-->>GUI: Text blocks
  GUI->>GUI: Show in docked reader pane
"""


def main() -> int:
    diagrams_dir = os.path.join(os.path.dirname(__file__), "diagrams")
    os.makedirs(diagrams_dir, exist_ok=True)
    arch_mmd = os.path.join(diagrams_dir, "architecture_flow.mmd")
    reader_mmd = os.path.join(diagrams_dir, "reader_sequence.mmd")
    open(arch_mmd, "w", encoding="utf-8").write(ARCH)
    open(reader_mmd, "w", encoding="utf-8").write(READER)
    # Render PNGs
    from .render_mermaid import main as render
    render(["--in", arch_mmd, "--out", os.path.join(diagrams_dir, "architecture_flow.png")])
    render(["--in", reader_mmd, "--out", os.path.join(diagrams_dir, "reader_sequence.png")])
    print("Generated diagrams in:", diagrams_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


