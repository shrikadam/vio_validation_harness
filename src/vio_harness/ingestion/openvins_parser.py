# src/vio_harness/ingestion/openvins_parser.py

import pandas as pd
import numpy as np
from pathlib import Path
from vio_harness.ingestion.base_parser import TrajectoryData, TrajectoryParser

class OpenVINSParser(TrajectoryParser):
    """
    Parses OpenVINS CSV output.
    Assumes headers: timestamp, px, py, pz, qx, qy, qz, qw
    """
    
    def parse(self, file_path: Path) -> TrajectoryData:
        if not file_path.exists():
            raise FileNotFoundError(f"OpenVINS log not found: {file_path}")
            
        df = pd.read_csv(file_path)
        
        # OpenVINS timestamps are typically already in seconds, 
        # but if they are in nanoseconds (> 1e17), scale them down.
        timestamps = df['timestamp'].to_numpy(dtype=np.float64)
        if timestamps[0] > 1e17:  
            timestamps /= 1e9
            
        positions = df[['px', 'py', 'pz']].to_numpy(dtype=np.float64)
        orientations = df[['qx', 'qy', 'qz', 'qw']].to_numpy(dtype=np.float64)
        
        return TrajectoryData(
            timestamps=timestamps,
            positions=positions,
            orientations=orientations
        )