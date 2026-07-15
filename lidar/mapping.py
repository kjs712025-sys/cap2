"""Mapping utilities for future navigation support."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(slots=True)
class MapCell:
    """Simple map cell placeholder."""

    occupied: bool = False
    cost: float = 1.0


@dataclass(slots=True)
class OccupancyMap:
    """Simple 2D occupancy map for future planning."""

    width: int = 100
    height: int = 100
    cells: dict[tuple[int, int], MapCell] = field(default_factory=dict)

    def mark(self, x: int, y: int, occupied: bool = True) -> None:
        """Mark a map location as occupied or free."""
        self.cells[(x, y)] = MapCell(occupied=occupied)
