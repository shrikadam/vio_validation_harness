# src/vio_harness/models/trajectory.py

import abc
from dataclasses import dataclass
from pathlib import Path
import numpy as np

# ==========================================
# MODELS (Unified Internal Structure)
# ==========================================

@dataclass
class TrajectoryData:
    """
    Unified internal data structure for 6-DoF trajectories.
    All ingestion adapters must return data in this format.
    """
    timestamps: np.ndarray    # Shape: (N,), dtype: float64 (Seconds)
    positions: np.ndarray     # Shape: (N, 3), dtype: float64 [x, y, z]
    orientations: np.ndarray  # Shape: (N, 4), dtype: float64 [qx, qy, qz, qw] (Hamilton)

    def __post_init__(self):
        """Basic validation to ensure arrays match in the temporal dimension."""
        n_frames = len(self.timestamps)
        assert self.positions.shape == (n_frames, 3), f"Expected positions shape {(n_frames, 3)}, got {self.positions.shape}"
        assert self.orientations.shape == (n_frames, 4), f"Expected orientations shape {(n_frames, 4)}, got {self.orientations.shape}"


# ==========================================
# PHASE 1: INGESTION (Adapter Pattern)
# ==========================================

class TrajectoryParser(abc.ABC):
    """
    Adapter interface for parsing various trajectory log formats.
    Concrete classes (e.g., OpenVINSParser) will implement this.
    """
    
    @abc.abstractmethod
    def parse(self, file_path: Path) -> TrajectoryData:
        """
        Reads a log file and normalizes it into the unified TrajectoryData format.
        
        Args:
            file_path (Path): Path to the log file (e.g., CSV, TXT, or HDF5).
            
        Returns:
            TrajectoryData: The normalized trajectory.
        """
        pass