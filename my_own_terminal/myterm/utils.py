from __future__ import annotations

import os
import subprocess
from typing import Dict, List, Tuple


def get_git_branch(cwd: str | None = None) -> str | None:
    try:
        result = subprocess.run(
            ["git", "rev-parse", "--abbrev-ref", "HEAD"],
            cwd=cwd,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=0.2,
            check=False,
        )
        branch = result.stdout.strip()
        return branch if branch and branch != "HEAD" else None
    except Exception:
        return None


def shorten_path(path: str, max_len: int = 40) -> str:
    if len(path) <= max_len:
        return path
    parts = path.split(os.sep)
    if not parts:
        return path
    result_parts: List[str] = []
    total = 0
    for i, part in enumerate(parts):
        if i == len(parts) - 1:
            result_parts.append(part)
            break
        first = part[0] if part else ""
        candidate = first + os.sep
        if total + len(candidate) + 1 < max_len:  # +1 for ellipsis later
            result_parts.append(first)
            total += len(candidate)
        else:
            result_parts.append("…")
            result_parts.append(parts[-1])
            return os.sep.join(result_parts)
    return os.sep.join(result_parts)


def which(executable: str, env: Dict[str, str] | None = None) -> str | None:
    search_path = (env or os.environ).get("PATH", "")
    for directory in search_path.split(os.pathsep):
        candidate = os.path.join(directory, executable)
        if os.path.isfile(candidate) and os.access(candidate, os.X_OK):
            return candidate
    return None
