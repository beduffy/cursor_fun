from __future__ import annotations

from my_own_browser.renderer import (
    render_for_terminal,
    truncate_text,
    render_html_to_terminal,
)


def test_truncate_text_basic():
    assert truncate_text("abcdef", max_chars=3) == "ab…"
    assert truncate_text("abc", max_chars=3) == "abc"
    assert truncate_text("abc", max_chars=0) == ""


def test_render_for_terminal_wraps_and_unescapes():
    text = "Hello &amp; goodbye"
    rendered = render_for_terminal([text], max_width=8)
    lines = rendered.splitlines()
    assert lines == ["Hello & ", "goodbye"]


def test_render_html_to_terminal_basic():
    html = """
    <html>
      <body>
        <h1>Title</h1>
        <p>Hello <a href="/x">world</a>!</p>
        <ul>
          <li>First item</li>
          <li>Second item</li>
        </ul>
      </body>
    </html>
    """
    out = render_html_to_terminal(html, base_url="https://example.com", max_width=40)
    assert "# Title" in out
    assert "Hello world [1]!" in out
    assert "- First item" in out
    assert "- Second item" in out
    assert "[1] https://example.com/x" in out


