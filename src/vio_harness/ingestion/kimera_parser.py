# src/vio_harness/ingestion/kimera_parser.py

import pandas as pd
import numpy as np
from pathlib import Path
from vio_harness.ingestion.base_parser import TrajectoryData, TrajectoryParser

class KimeraParser(TrajectoryParser):
    """
    Parses Kimera-VIO CSV output.
    Assumes headers: timestamp, x, y, z, qx, qy, qz, qw
    """
    
    def parse(self, file_path: Path) -> TrajectoryData:
        if not file_path.exists():
            raise FileNotFoundError(f"Kimera-VIO log not found: {file_path}")
            
        df = pd.read_csv(file_path)
        
        # Convert Kimera's nanosecond timestamps to seconds
        timestamps = df['timestamp'].to_numpy(dtype=np.float64)
        if timestamps[0] > 1e17 or np.mean(np.diff(timestamps)) > 1e6:
            timestamps /= 1e9
            
        # Note: Map Kimera's specific column names to the expected unified format
        positions = df[['x', 'y', 'z']].to_numpy(dtype=np.float64)
        orientations = df[['qx', 'qy', 'qz', 'qw']].to_numpy(dtype=np.float64)
        
        return TrajectoryData(
            timestamps=timestamps,
            positions=positions,
            orientations=orientations
        )