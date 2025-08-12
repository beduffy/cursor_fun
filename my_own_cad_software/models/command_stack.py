from __future__ import annotations

from dataclasses import dataclass
from typing import List, Protocol


class Command(Protocol):
  def do(self) -> None: ...
  def undo(self) -> None: ...


@dataclass
class CommandStack:
  _stack: List[Command]
  _index: int = -1

  def __init__(self) -> None:
    self._stack = []
    self._index = -1

  def push_and_do(self, cmd: Command) -> None:
    # drop any redoable commands
    del self._stack[self._index + 1 :]
    self._stack.append(cmd)
    self._index += 1
    cmd.do()

  def can_undo(self) -> bool:
    return self._index >= 0

  def can_redo(self) -> bool:
    return self._index + 1 < len(self._stack)

  def undo(self) -> None:
    if not self.can_undo():
      return
    self._stack[self._index].undo()
    self._index -= 1

  def redo(self) -> None:
    if not self.can_redo():
      return
    self._index += 1
    self._stack[self._index].do()
