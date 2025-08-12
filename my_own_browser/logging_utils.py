from __future__ import annotations

import logging
from logging.handlers import RotatingFileHandler
import os
from typing import Optional

from .storage import get_app_paths


def get_log_path(app_name: str = "my_own_browser") -> str:
    paths = get_app_paths(app_name)
    log_dir = os.path.join(paths.config_dir, "logs")
    try:
        os.makedirs(log_dir, exist_ok=True)
    except Exception:
        pass
    return os.path.join(log_dir, "app.log")


def get_app_logger(name: str = "my_own_browser", level: int = logging.INFO, app_name: Optional[str] = None) -> logging.Logger:
    logger = logging.getLogger(name)
    if logger.handlers:
        return logger

    logger.setLevel(level)
    resolved_app = app_name if app_name is not None else (name.split(".")[0] if name else "my_own_browser")
    log_path = get_log_path(resolved_app)
    handler = RotatingFileHandler(log_path, maxBytes=512 * 1024, backupCount=3)
    fmt = logging.Formatter("%(asctime)s %(levelname)s %(name)s: %(message)s")
    handler.setFormatter(fmt)
    logger.addHandler(handler)
    return logger


__all__ = ["get_app_logger", "get_log_path"]


