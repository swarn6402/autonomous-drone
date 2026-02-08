"""
Telemetry Manager Module

Handles logging and tracking of flight telemetry data.
Provides real-time telemetry streaming capabilities.
"""

from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Dict, Optional


@dataclass
class TelemetryFrame:
    """A single telemetry data frame."""
    timestamp: datetime
    imu_data: Optional[Dict[str, float]] = None
    gps_data: Optional[Dict[str, float]] = None
    barometer_data: Optional[Dict[str, float]] = None
    flight_state: Optional[str] = None
    battery_voltage: float = 0.0
    battery_current: float = 0.0
    custom_data: Dict[str, Any] = field(default_factory=dict)


class TelemetryManager:
    """
    Manages telemetry collection and logging.
    
    Collects flight data, manages logging, and provides
    real-time telemetry streaming.
    """
    
    def __init__(self, buffer_size: int = 1000):
        """
        Initialize the telemetry manager.
        
        Args:
            buffer_size: Maximum number of telemetry frames to buffer
        """
        self.buffer_size = buffer_size
        self.frames: list[TelemetryFrame] = []
        self.is_logging = False
        self.log_file: Optional[str] = None
    
    def start_logging(self, filename: str = "flight_log.csv"):
        """
        Start logging telemetry to file.
        
        Args:
            filename: Output log filename
        """
        self.log_file = filename
        self.is_logging = True
        # Placeholder for file initialization
    
    def stop_logging(self):
        """Stop logging and close log file."""
        self.is_logging = False
        # Placeholder for file closure
    
    def record_frame(self, frame: TelemetryFrame):
        """
        Record a telemetry frame.
        
        Args:
            frame: Telemetry data frame to record
        """
        self.frames.append(frame)
        
        # Keep buffer size manageable
        if len(self.frames) > self.buffer_size:
            self.frames.pop(0)
        
        if self.is_logging:
            self._write_frame_to_file(frame)
    
    def _write_frame_to_file(self, frame: TelemetryFrame):
        """Write a frame to the log file."""
        # Placeholder for file writing
        pass
    
    def get_last_frame(self) -> Optional[TelemetryFrame]:
        """Get the last recorded telemetry frame."""
        return self.frames[-1] if self.frames else None
    
    def get_frames(self, count: int = 10) -> list[TelemetryFrame]:
        """
        Get the last N telemetry frames.
        
        Args:
            count: Number of recent frames to retrieve
            
        Returns:
            List of telemetry frames
        """
        return self.frames[-count:]
    
    def clear_buffer(self):
        """Clear the telemetry buffer."""
        self.frames.clear()
