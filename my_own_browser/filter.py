from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from typing import List

from .storage import get_app_paths


@dataclass
class Blocklist:
    patterns: List[str] = field(default_factory=list)

    def matches(self, url: str) -> bool:
        candidate = (url or "").lower()
        for pattern in self.patterns:
            p = pattern.strip().lower()
            if not p:
                continue
            if p in candidate:
                return True
        return False


def _blocklist_path(app_name: str = "my_own_browser") -> str:
    paths = get_app_paths(app_name)
    return os.path.join(paths.config_dir, "blocklist.json")


def load_blocklist(app_name: str = "my_own_browser") -> Blocklist:
    path = _blocklist_path(app_name)
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
            items = [str(x) for x in (data or [])]
            return Blocklist(patterns=items)
    except Exception:
        # Provide a tiny sane default
        return Blocklist(patterns=["/ads/", "doubleclick.net", "googlesyndication.com"]) 


def save_blocklist(blocklist: Blocklist, app_name: str = "my_own_browser") -> None:
    path = _blocklist_path(app_name)
    try:
        with open(path, "w", encoding="utf-8") as f:
            json.dump(blocklist.patterns, f, ensure_ascii=False, indent=2)
    except Exception:
        pass


__all__ = ["Blocklist", "load_blocklist", "save_blocklist"]


