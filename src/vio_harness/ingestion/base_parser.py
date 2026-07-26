# src/vio_harness/ingestion/base_parser.py

import abc
from pathlib import Path
from vio_harness.models.trajectory import TrajectoryData

class TrajectoryParser(abc.ABC):
    """
    Adapter interface for parsing various trajectory log formats.
    """
    
    @abc.abstractmethod
    def parse(self, file_path: Path) -> TrajectoryData:
        """Reads a log file and normalizes it into TrajectoryData."""
        pass