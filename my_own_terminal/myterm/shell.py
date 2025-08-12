from __future__ import annotations

import os
import shlex
import subprocess
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from .builtins import CommandResult, builtins
from .utils import get_git_branch, shorten_path, which


@dataclass
class ShellConfig:
    aliases: Dict[str, str] = field(default_factory=lambda: {
        "ll": "ls -alF",
        "la": "ls -A",
        "l": "ls -CF",
        "gs": "git status",
        "gd": "git diff",
    })
    env: Dict[str, str] = field(default_factory=lambda: dict(os.environ))
    prompt_color: str = "\033[1;36m"  # bright cyan
    reset_color: str = "\033[0m"


class MyTerm:
    def __init__(self, config: Optional[ShellConfig] = None) -> None:
        self.config = config or ShellConfig()
        self.aliases: Dict[str, str] = dict(self.config.aliases)
        self._setup_history()

    # ---------- Core loop ----------
    def run(self) -> int:
        # Late import to avoid hard dependency if not on POSIX
        try:
            import readline  # type: ignore

            from .completion import Completer

            Completer(self.get_aliases)
        except Exception:
            pass

        while True:
            try:
                line = input(self._render_prompt())
            except EOFError:
                print()
                return 0
            except KeyboardInterrupt:
                print()
                continue

            if not line.strip():
                continue

            try:
                code = self.execute_line(line)
                # Persist history after each command
                try:
                    import readline as _readline  # type: ignore

                    _readline.write_history_file(self.history_path)
                except Exception:
                    pass
            except SystemExit as exit_exc:
                return int(exit_exc.code) if hasattr(exit_exc, "code") else 0
            except Exception as exc:
                print(f"error: {exc}")
                code = 1

    def get_aliases(self) -> Dict[str, str]:
        return self.aliases

    # ---------- Prompt ----------
    def _render_prompt(self) -> str:
        cwd = os.getcwd()
        branch = get_git_branch(cwd)
        short = shorten_path(cwd)
        parts: List[str] = []
        parts.append(f"{self.config.prompt_color}{short}{self.config.reset_color}")
        if branch:
            parts.append(f"\033[33m({branch})\033[0m")  # yellow branch
        parts.append("$ ")
        return " ".join(parts)

    # ---------- Execution ----------
    def execute_line(self, line: str) -> int:
        # Support `alias name='value'` and `alias name=value`
        if line.startswith("alias "):
            assignment = line[len("alias ") :].strip()
            if not assignment:
                for name, value in sorted(self.aliases.items()):
                    print(f"alias {name}='{value}'")
                return 0
            if "=" in assignment:
                name, value = assignment.split("=", 1)
                value = value.strip().strip("'\"")
                self.aliases[name.strip()] = value
                return 0

        tokens = shlex.split(line)
        if not tokens:
            return 0

        cmd, args = tokens[0], tokens[1:]

        # Alias expansion (single pass)
        if cmd in self.aliases:
            expanded = self.aliases[cmd]
            expanded_line = expanded + (" " + " ".join(shlex.quote(a) for a in args) if args else "")
            return self.execute_line(expanded_line)

        # Built-in commands
        builtin = builtins.get(cmd)
        if builtin:
            result = builtin(args)
            if result.output:
                sys.stdout.write(result.output)
                sys.stdout.flush()
            return result.return_code

        # External command via subprocess
        return self._run_external(tokens, raw_line=line)

    # ---------- History ----------
    def _setup_history(self) -> None:
        self.history_path = os.path.join(os.path.expanduser("~"), ".myterm_history")
        try:
            import readline  # type: ignore

            if os.path.exists(self.history_path):
                try:
                    readline.read_history_file(self.history_path)
                except Exception:
                    pass
            readline.set_history_length(1000)
        except Exception:
            self.history_path = os.devnull

    def _run_external(self, tokens: List[str], raw_line: Optional[str] = None) -> int:
        # Always route through bash so that pipes, globs, redirects,
        # variables, and builtins behave as expected.
        command_str = raw_line if raw_line is not None else " ".join(shlex.quote(t) for t in tokens)
        try:
            completed = subprocess.run(
                command_str,
                env=self.config.env,
                shell=True,
                executable="/bin/bash",
            )
            return completed.returncode
        except FileNotFoundError:
            print("command not found")
            return 127
        except PermissionError:
            print("permission denied")
            return 126


__all__ = ["MyTerm", "ShellConfig"]
