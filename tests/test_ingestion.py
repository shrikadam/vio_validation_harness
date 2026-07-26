# tests/test_ingestion.py

import pytest
import numpy as np
import pandas as pd
from pathlib import Path
from vio_harness.ingestion.openvins_parser import OpenVINSParser
from vio_harness.models.trajectory import TrajectoryData

def test_openvins_parser_validates_shapes(tmp_path):
    """Tests if the OpenVINS adapter correctly builds the TrajectoryData."""
    # Create a mock CSV
    csv_file = tmp_path / "mock_openvins.csv"
    
    mock_data = {
        "timestamp": [1.0, 1.1, 1.2],
        "px": [0.0, 1.0, 2.0], "py": [0.0, 0.0, 0.0], "pz": [0.0, 0.0, 0.0],
        "qx": [0.0, 0.0, 0.0], "qy": [0.0, 0.0, 0.0], "qz": [0.0, 0.0, 0.0], "qw": [1.0, 1.0, 1.0]
    }
    pd.DataFrame(mock_data).to_csv(csv_file, index=False)
    
    # Parse
    parser = OpenVINSParser()
    data = parser.parse(csv_file)
    
    # Assert Type and Shapes
    assert isinstance(data, TrajectoryData)
    assert data.positions.shape == (3, 3)
    assert data.orientations.shape == (3, 4)
    assert data.timestamps.shape == (3,)