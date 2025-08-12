"""My Own CAD Software (2D)

Core modules for a lightweight, educational 2D CAD system.
The package is structured so that the core is testable without any GUI.
"""

from .models.document import Document
from .models.entities import LineEntity, CircleEntity, RectEntity

__all__ = [
    "Document",
    "LineEntity",
    "CircleEntity",
    "RectEntity",
]
