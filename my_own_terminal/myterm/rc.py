from __future__ import annotations

import os
from typing import Dict


def load_rc(alias_target: Dict[str, str], env_target: Dict[str, str]) -> None:
    path = os.path.join(os.path.expanduser("~"), ".mytermrc")
    if not os.path.exists(path):
        return
    try:
        with open(path, "r", encoding="utf-8") as f:
            for raw in f:
                line = raw.strip()
                if not line or line.startswith("#"):
                    continue
                if line.startswith("alias ") and "=" in line:
                    name, val = line[len("alias ") :].split("=", 1)
                    alias_target[name.strip()] = val.strip().strip("'\"")
                elif line.startswith("export ") and "=" in line:
                    name, val = line[len("export ") :].split("=", 1)
                    name = name.strip()
                    val = val.strip().strip("'\"")
                    os.environ[name] = val
                    env_target[name] = val
    except Exception:
        # Silent failure to avoid blocking startup
        return
