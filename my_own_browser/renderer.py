from __future__ import annotations

from html import unescape
from typing import Iterable
from urllib.parse import urljoin

from bs4 import BeautifulSoup


def truncate_text(text: str, max_chars: int = 2000) -> str:
    if max_chars <= 0:
        return ""
    if len(text) <= max_chars:
        return text
    return text[: max_chars - 1] + "…"




def render_for_terminal(lines: Iterable[str], max_width: int = 100) -> str:
    wrapped_lines = []
    for raw in lines:
        text = unescape(raw).replace("\r", "")
        while text:
            wrapped_lines.append(text[:max_width])
            text = text[max_width:]
    return "\n".join(wrapped_lines)


def _wrap_paragraph(text: str, max_width: int) -> list[str]:
    words = unescape(text or "").split()
    if not words:
        return [""]

    lines: list[str] = []
    current: list[str] = []
    current_len = 0

    for word in words:
        if current_len == 0:
            current = [word]
            current_len = len(word)
        elif current_len + 1 + len(word) <= max_width:
            current.append(word)
            current_len += 1 + len(word)
        else:
            lines.append(" ".join(current))
            current = [word]
            current_len = len(word)

    if current:
        lines.append(" ".join(current))
    return lines


def render_html_to_terminal(html: str, base_url: str = "", max_width: int = 100, max_chars: int = 4000) -> str:
    """Render a small subset of HTML into a terminal-friendly string.

    Features:
    - Headings h1..h6 as markdown-like '#'
    - Paragraphs with wrapped text
    - Unordered/ordered lists
    - Links as inline text with numeric footnotes, listed at bottom
    - Pre/code blocks preserved (wrapped to width but without reflow)
    - Images rendered as [IMG alt] or [IMG src]
    """
    soup = BeautifulSoup(html or "", "html.parser")

    # Resolve and annotate links with footnote numbers
    footnotes: list[str] = []
    for idx, a in enumerate(soup.find_all("a", href=True), start=1):
        href = urljoin(base_url, a["href"]) if base_url else a["href"]
        text = a.get_text(strip=True) or href
        a.replace_with(f"{text} [{idx}]")
        footnotes.append(href)

    body = soup.body or soup
    lines: list[str] = []

    def add_blank_line():
        if lines and lines[-1] != "":
            lines.append("")

    def emit_paragraph(text: str):
        for ln in _wrap_paragraph(text, max_width):
            lines.append(ln)

    def emit_pre(text: str):
        for raw in (text or "").replace("\r", "").splitlines():
            # hard wrap pre lines without reflow
            temp = raw
            while temp:
                lines.append(temp[:max_width])
                temp = temp[max_width:]

    block_tags = {
        "p", "div", "section", "article", "main", "header", "footer", "aside",
        "nav", "blockquote",
    }

    def emit_node(node):
        name = getattr(node, "name", None)
        if not name:
            text = str(node)
            if text.strip():
                emit_paragraph(text)
            return

        if name in {"style", "script", "noscript"}:
            return

        if name in {"h1", "h2", "h3", "h4", "h5", "h6"}:
            level = int(name[1])
            add_blank_line()
            heading_text = node.get_text(" ", strip=True)
            lines.append(f"{'#' * level} {heading_text}")
            add_blank_line()
            return

        if name in {"ul", "ol"}:
            add_blank_line()
            is_ordered = name == "ol"
            start = 1
            for li in node.find_all("li", recursive=False):
                bullet = f"{start}. " if is_ordered else "- "
                text = li.get_text(" ", strip=True)
                for i, ln in enumerate(_wrap_paragraph(text, max_width - len(bullet))):
                    prefix = bullet if i == 0 else " " * len(bullet)
                    lines.append(prefix + ln)
                start += 1
            add_blank_line()
            return

        if name == "pre":
            add_blank_line()
            emit_pre(node.get_text())
            add_blank_line()
            return

        if name == "img":
            alt = node.get("alt")
            src = node.get("src")
            label = alt or src or "image"
            emit_paragraph(f"[IMG {label}]")
            return

        if name == "br":
            lines.append("")
            return

        if name in block_tags:
            add_blank_line()
            emit_paragraph(node.get_text(" ", strip=True))
            add_blank_line()
            return

        # Fallback: emit text content
        emit_paragraph(node.get_text(" ", strip=True))

    # Traverse top-level body children to maintain some structure
    for child in body.children:
        emit_node(child)

    if footnotes:
        lines.append("")
        lines.append("Links:")
        for i, href in enumerate(footnotes, start=1):
            lines.append(f"  [{i}] {href}")

    rendered = "\n".join(lines).strip()
    return truncate_text(rendered, max_chars)


__all__ = [
    "truncate_text",
    "render_for_terminal",
    "render_html_to_terminal",
]


