from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional
from urllib.parse import urljoin, urlparse

import requests
from bs4 import BeautifulSoup


@dataclass
class Page:
    url: str
    status_code: int
    content: str
    text: str
    title: Optional[str]
    links: List[str] = field(default_factory=list)


 

class Browser:
    """A minimal stateful browser with history and simple rendering."""

    def __init__(self, user_agent: str = "my-own-browser/0.1") -> None:
        self._session = requests.Session()
        self._session.headers.update({"User-Agent": user_agent})
        self._history: List[Page] = []
        self._forward_stack: List[Page] = []

    @property
    def history(self) -> List[Page]:
        return list(self._history)

    def current_page(self) -> Optional[Page]:
        return self._history[-1] if self._history else None

    def go(self, url: str, timeout: float = 15.0) -> Page:
        normalized = self._normalize_url(url)
        response = self._session.get(normalized, timeout=timeout)
        response.raise_for_status()

        page = self._build_page(normalized, response.status_code, response.text)
        self._history.append(page)
        self._forward_stack.clear()
        return page

    def back(self) -> Optional[Page]:
        if len(self._history) <= 1:
            return self.current_page()
        last = self._history.pop()
        self._forward_stack.append(last)
        return self.current_page()

    def forward(self) -> Optional[Page]:
        if not self._forward_stack:
            return self.current_page()
        page = self._forward_stack.pop()
        self._history.append(page)
        return page

    def _build_page(self, url: str, status_code: int, html: str) -> Page:
        soup = BeautifulSoup(html, "html.parser")
        title_tag = soup.find("title")
        title = title_tag.text.strip() if title_tag else None

        # Extract and absolutize links
        links: List[str] = []
        for a in soup.find_all("a", href=True):
            absolute = urljoin(url, a["href"])  # resolves relative URLs
            links.append(absolute)

        text = soup.get_text(" ", strip=True)
        return Page(url=url, status_code=status_code, content=html, text=text, title=title, links=links)

    def _normalize_url(self, url: str) -> str:
        parsed = urlparse(url)
        if not parsed.scheme:
            return f"https://{url}"
        return url


__all__ = ["Browser", "Page"]


