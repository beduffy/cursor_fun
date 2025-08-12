from __future__ import annotations

import os
from dataclasses import dataclass
import json
from typing import Any, Dict


@dataclass
class AppPaths:
    config_dir: str
    cache_dir: str
    downloads_dir: str


def get_app_paths(app_name: str = "my_own_browser") -> AppPaths:
    home = os.path.expanduser("~")
    base_config = os.path.join(home, ".config", app_name)
    base_cache = os.path.join(home, ".cache", app_name)
    downloads = os.path.join(home, "Downloads")

    for path in (base_config, base_cache, downloads):
        try:
            os.makedirs(path, exist_ok=True)
        except Exception:
            # Best-effort; caller should handle missing dirs gracefully
            pass

    return AppPaths(config_dir=base_config, cache_dir=base_cache, downloads_dir=downloads)


__all__ = ["get_app_paths", "AppPaths"]


def get_settings_path(app_name: str = "my_own_browser") -> str:
    paths = get_app_paths(app_name)
    return os.path.join(paths.config_dir, "settings.json")


def load_settings(app_name: str = "my_own_browser") -> Dict[str, Any]:
    path = get_settings_path(app_name)
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return {}


def save_settings(settings: Dict[str, Any], app_name: str = "my_own_browser") -> None:
    path = get_settings_path(app_name)
    try:
        with open(path, "w", encoding="utf-8") as f:
            json.dump(settings, f, ensure_ascii=False, indent=2)
    except Exception:
        pass



