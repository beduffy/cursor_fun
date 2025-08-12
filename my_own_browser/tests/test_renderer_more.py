from __future__ import annotations

from my_own_browser.renderer import render_html_to_terminal


def test_render_html_handles_images_and_pre():
    html = """
    <html><body>
    <h2>Code</h2>
    <pre>for i in range(3):\n    print(i)</pre>
    <p>Picture: <img src="/pic.png" alt="Logo"></p>
    </body></html>
    """
    out = render_html_to_terminal(html, base_url="https://ex.com", max_width=40)
    assert "## Code" in out
    assert "for i in range(3):" in out
    assert "print(i)" in out
    assert "[IMG Logo]" in out


def test_render_html_inline_styles():
    html = """
    <p>Make it <strong>bold</strong> and <em>emph</em> and <code>code</code>.</p>
    """
    out = render_html_to_terminal(html, base_url="https://ex.com", max_width=80)
    assert "**bold**" in out
    assert "_emph_" in out
    assert "`code`" in out



