# src/vio_harness/fault_injection/base_injector.py

import abc
import pandas as pd

class BaseTamper(abc.ABC):
    """
    Abstract base class for injecting hardware faults into raw sensor data.
    """
    
    @abc.abstractmethod
    def apply(self, sensor_data: pd.DataFrame) -> pd.DataFrame:
        """
        Applies a mathematical fault to the sensor data.
        
        Args:
            sensor_data (pd.DataFrame): The raw sensor data.
            
        Returns:
            pd.DataFrame: The tampered sensor data.
        """
        pass