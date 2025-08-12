# My Own CAD Software (2D)

A lightweight, educational 2D CAD application implemented in Python. The codebase is split into a tested core (geometry, entities, document, commands, serialization) and an optional GUI layer (PySide6) for interactive drawing.

## Features (initial)
- Line, circle, rectangle entities (model-only; GUI supports line initially)
- Hit-testing and bounding boxes
- Grid snapping utilities
- Document model with add/remove/move
- Undo/redo via command stack
- JSON save/load
- Unit tests for geometry and document logic

## Roadmap
- More drawing tools (arc, polyline, bezier)
- Layers, colors, line styles
- Constraints and dimensions
- Advanced snaps (end/mid/center/orthogonal/intersection)
- Selection and transform gizmos (GUI)
- DXF/SVG import/export

## Install
If using conda, create or activate your environment first.

```bash
# Optional: conda
# conda create -n mycad python=3.11 -y && conda activate mycad

pip install -r my_own_cad_software/requirements.txt
```

## Run (GUI - optional)
```bash
python -m my_own_cad_software.cad_app
```

If PySide6 is not installed or a display is not available, you can still run the unit tests which exercise the core logic only.

## Tests
```bash
pytest -q my_own_cad_software/tests
```

## Package layout
```
my_own_cad_software/
  cad_app.py                # Optional GUI entrypoint (PySide6)
  geometry.py               # Pure geometry helpers and hit-testing
  __init__.py
  io/
    serializer.py           # JSON save/load for document and entities
  models/
    command_stack.py        # Undo/redo command pattern
    document.py             # In-memory document holding entities
    entities.py             # Entity definitions: Line, Circle, Rect
  tests/
    test_geometry.py
    test_document.py
    test_commands.py
  requirements.txt
  README.md
```

## Notes
- The core is pure Python, no GUI dependency. This keeps tests fast and reliable.
- The GUI is intentionally minimal for now; it will evolve as features land in the core.
