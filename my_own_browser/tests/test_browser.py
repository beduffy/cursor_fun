from __future__ import annotations

from my_own_browser.browser import Browser


def test_go_fetches_and_parses(httpserver):
    html = """
    <html>
      <head><title>Hello</title></head>
      <body>
        <p>Welcome <a href="/next">next</a></p>
      </body>
    </html>
    """
    httpserver.expect_request("/").respond_with_data(html, content_type="text/html")
    base_url = httpserver.url_for("/")

    browser = Browser(user_agent="test-agent")
    page = browser.go(base_url)

    assert page.status_code == 200
    assert page.title == "Hello"
    assert "Welcome" in page.text
    assert page.links and page.links[0].endswith("/next")


def test_history_back_and_forward(httpserver):
    httpserver.expect_request("/").respond_with_data("<title>Root</title>")
    httpserver.expect_request("/a").respond_with_data("<title>A</title>")
    httpserver.expect_request("/b").respond_with_data("<title>B</title>")
    base = httpserver.url_for("/")

    browser = Browser()
    page_root = browser.go(base)
    page_a = browser.go(base + "a")
    page_b = browser.go(base + "b")

    assert [p.title for p in browser.history] == ["Root", "A", "B"]

    browser.back()
    assert browser.current_page().title == "A"

    browser.back()
    assert browser.current_page().title == "Root"

    browser.forward()
    assert browser.current_page().title == "A"

    browser.forward()
    assert browser.current_page().title == "B"


