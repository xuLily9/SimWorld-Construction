"""Typed HRC observations and simulator-independent proximity calculations."""

from .hrc_state import HRCState, Point2D
from .state_extractor import StateExtractor, calculate_state

__all__ = ["HRCState", "Point2D", "StateExtractor", "calculate_state"]
