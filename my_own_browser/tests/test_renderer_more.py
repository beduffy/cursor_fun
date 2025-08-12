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



