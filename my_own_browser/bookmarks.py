from __future__ import annotations

import json
import os
from dataclasses import dataclass
from typing import List

from .storage import get_app_paths


@dataclass
class Bookmark:
    title: str
    url: str


def _bookmarks_path(app_name: str = "my_own_browser") -> str:
    paths = get_app_paths(app_name)
    return os.path.join(paths.config_dir, "bookmarks.json")


def load_bookmarks(app_name: str = "my_own_browser") -> List[Bookmark]:
    path = _bookmarks_path(app_name)
    try:
        with open(path, "r", encoding="utf-8") as f:
            raw = json.load(f)
            items = []
            for it in raw or []:
                title = it.get("title") or it.get("name") or it.get("url") or "(untitled)"
                url = it.get("url") or ""
                if url:
                    items.append(Bookmark(title=title, url=url))
            return items
    except Exception:
        return []


def save_bookmarks(bookmarks: List[Bookmark], app_name: str = "my_own_browser") -> None:
    path = _bookmarks_path(app_name)
    try:
        data = [{"title": b.title, "url": b.url} for b in bookmarks]
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
    except Exception:
        pass


def add_bookmark(url: str, title: str, app_name: str = "my_own_browser") -> Bookmark:
    items = load_bookmarks(app_name)
    # Deduplicate by URL
    for it in items:
        if it.url == url:
            return it
    new_bm = Bookmark(title=title or url, url=url)
    items.append(new_bm)
    save_bookmarks(items, app_name)
    return new_bm


__all__ = [
    "Bookmark",
    "load_bookmarks",
    "save_bookmarks",
    "add_bookmark",
]


