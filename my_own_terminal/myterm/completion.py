from __future__ import annotations

import os
import readline
from typing import Dict, List


class Completer:
    def __init__(self, get_aliases):
        self.get_aliases = get_aliases
        readline.parse_and_bind("tab: complete")
        readline.set_completer(self.complete)

    def complete(self, text: str, state: int):
        buffer = readline.get_line_buffer()
        tokens = buffer.split()
        suggestions: List[str] = []

        # First token: command/alias/executable
        if len(tokens) <= 1 or (buffer and buffer[-1].isspace()):
            suggestions.extend(self._complete_command(text))
        # Other tokens: file system paths
        suggestions.extend(self._complete_path(text))

        suggestions = sorted(set(suggestions))
        return suggestions[state] if state < len(suggestions) else None

    def _complete_command(self, text: str) -> List[str]:
        # Aliases
        suggestions = [name for name in self.get_aliases().keys() if name.startswith(text)]

        # Builtins
        from .builtins import builtins

        suggestions.extend([name for name in builtins.names() if name.startswith(text)])

        # PATH executables (simple scan of names only)
        for directory in os.environ.get("PATH", "").split(os.pathsep):
            if not os.path.isdir(directory):
                continue
            try:
                for entry in os.listdir(directory):
                    if entry.startswith(text):
                        suggestions.append(entry)
            except Exception:
                pass
        return suggestions

    def _complete_path(self, text: str) -> List[str]:
        if not text:
            text = ""
        dirname = os.path.dirname(text) or "."
        prefix = os.path.basename(text)
        try:
            entries = os.listdir(os.path.expanduser(dirname))
        except Exception:
            return []
        results: List[str] = []
        for entry in entries:
            if not entry.startswith(prefix):
                continue
            full = os.path.join(dirname, entry)
            if os.path.isdir(os.path.expanduser(full)):
                results.append(full + "/")
            else:
                results.append(full)
        return results
