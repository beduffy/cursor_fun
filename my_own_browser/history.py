from __future__ import annotations

import json
import os
from dataclasses import dataclass
from typing import List

from .storage import get_app_paths


@dataclass
class HistoryEntry:
    title: str
    url: str


def _history_path(app_name: str = "my_own_browser") -> str:
    paths = get_app_paths(app_name)
    return os.path.join(paths.config_dir, "history.json")


def load_history(app_name: str = "my_own_browser") -> List[HistoryEntry]:
    path = _history_path(app_name)
    try:
        with open(path, "r", encoding="utf-8") as f:
            raw = json.load(f) or []
            entries: List[HistoryEntry] = []
            for it in raw:
                url = it.get("url") or ""
                if not url:
                    continue
                title = it.get("title") or url
                entries.append(HistoryEntry(title=title, url=url))
            return entries
    except Exception:
        return []


def save_history(entries: List[HistoryEntry], app_name: str = "my_own_browser") -> None:
    path = _history_path(app_name)
    try:
        data = [{"title": e.title, "url": e.url} for e in entries]
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
    except Exception:
        pass


def add_to_history(url: str, title: str, app_name: str = "my_own_browser", max_entries: int = 500) -> HistoryEntry:
    title = title or url
    entries = load_history(app_name)
    # Avoid duplicates in a row
    if entries and entries[-1].url == url:
        return entries[-1]
    entry = HistoryEntry(title=title, url=url)
    entries.append(entry)
    if len(entries) > max_entries:
        entries = entries[-max_entries:]
    save_history(entries, app_name)
    return entry


__all__ = [
    "HistoryEntry",
    "load_history",
    "save_history",
    "add_to_history",
]


