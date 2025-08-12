from __future__ import annotations

import os
import shlex
import sys
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Tuple


@dataclass
class CommandResult:
    return_code: int
    output: str = ""


class BuiltinRegistry:
    def __init__(self) -> None:
        self._registry: Dict[str, Callable[[List[str]], CommandResult]] = {}

    def register(self, name: str):
        def _decorator(func: Callable[[List[str]], CommandResult]):
            self._registry[name] = func
            return func

        return _decorator

    def get(self, name: str) -> Optional[Callable[[List[str]], CommandResult]]:
        return self._registry.get(name)

    def names(self) -> List[str]:
        return sorted(self._registry.keys())


builtins = BuiltinRegistry()


@builtins.register("cd")
def builtin_cd(args: List[str]) -> CommandResult:
    target = args[0] if args else os.path.expanduser("~")
    try:
        os.chdir(os.path.expanduser(target))
        return CommandResult(0)
    except FileNotFoundError:
        return CommandResult(1, f"cd: no such file or directory: {target}\n")
    except NotADirectoryError:
        return CommandResult(1, f"cd: not a directory: {target}\n")
    except PermissionError:
        return CommandResult(1, f"cd: permission denied: {target}\n")


@builtins.register("pwd")
def builtin_pwd(_: List[str]) -> CommandResult:
    return CommandResult(0, os.getcwd() + "\n")


@builtins.register("echo")
def builtin_echo(args: List[str]) -> CommandResult:
    return CommandResult(0, " ".join(args) + "\n")


@builtins.register("exit")
def builtin_exit(args: List[str]) -> CommandResult:
    code = int(args[0]) if args else 0
    # Use sys.exit to escape the REPL
    raise SystemExit(code)


@builtins.register("help")
def builtin_help(_: List[str]) -> CommandResult:
    return CommandResult(0, "Builtins: " + ", ".join(builtins.names()) + "\n")


@builtins.register("which")
def builtin_which(args: List[str]) -> CommandResult:
    from .utils import which

    outputs: List[str] = []
    for name in args:
        path = which(name)
        if path:
            outputs.append(path)
    return CommandResult(0, ("\n".join(outputs) + ("\n" if outputs else "")))


@builtins.register("alias")
def builtin_alias(args: List[str]) -> CommandResult:
    # Handled by MyTerm; placeholder for completion
    return CommandResult(0)
