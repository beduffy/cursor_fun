"""A tiny, testable toy browser package.

This package provides a minimal browser core with:
- HTTP fetching
- Simple HTML-to-text rendering
- Link extraction with absolute URL resolution
- In-memory history navigation

The goal is to iterate feature-by-feature with tests.
"""

__all__ = [
    "browser",
    "renderer",
]


