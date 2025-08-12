from __future__ import annotations

import os

from my_own_browser.logging_utils import get_app_logger, get_log_path


def test_logger_writes(tmp_path, monkeypatch):
    fake_home = tmp_path / "home"
    fake_home.mkdir()
    monkeypatch.setenv("HOME", str(fake_home))

    log_path = get_log_path("test_logs")
    logger = get_app_logger("test_logs", app_name="test_logs")
    logger.info("hello world")
    logger.handlers[0].flush()
    assert os.path.isfile(log_path)
    data = open(log_path, "r", encoding="utf-8").read()
    assert "hello world" in data


