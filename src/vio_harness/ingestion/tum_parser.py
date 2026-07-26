# src/vio_harness/ingestion/tum_parser.py

import pandas as pd
import numpy as np
from pathlib import Path
from vio_harness.ingestion.base_parser import TrajectoryParser
from vio_harness.models.trajectory import TrajectoryData

class TUMParser(TrajectoryParser):
    """
    Parses trajectory files in the standard TUM RGB-D format:
    [timestamp tx ty tz qx qy qz qw]
    """
    
    def parse(self, file_path: Path) -> TrajectoryData:
        if not file_path.exists():
            raise FileNotFoundError(f"Trajectory file not found: {file_path}")
            
        # Read the space-separated file, ignoring comment lines starting with '#'
        df = pd.read_csv(
            file_path, 
            sep=r'\s+', 
            comment='#', 
            names=['timestamp', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw']
        )
        
        # Drop any malformed rows
        df.dropna(inplace=True)
        
        # Extract to optimized NumPy arrays
        timestamps = df['timestamp'].to_numpy(dtype=np.float64)
        positions = df[['tx', 'ty', 'tz']].to_numpy(dtype=np.float64)
        orientations = df[['qx', 'qy', 'qz', 'qw']].to_numpy(dtype=np.float64)
        
        return TrajectoryData(
            timestamps=timestamps,
            positions=positions,
            orientations=orientations
        )