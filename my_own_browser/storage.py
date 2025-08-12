from __future__ import annotations

import os
from dataclasses import dataclass


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



