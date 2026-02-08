"""
GPS Module

Handles GPS data collection and position tracking.
"""

from dataclasses import dataclass
from typing import Optional


@dataclass
class GPSData:
    """GPS position and velocity data."""
    latitude: float  # degrees
    longitude: float  # degrees
    altitude: float  # meters above sea level
    hdop: float  # Horizontal dilution of precision
    num_satellites: int
    velocity_x: float  # m/s (North)
    velocity_y: float  # m/s (East)
    velocity_z: float  # m/s (Down)
    is_fixed: bool  # Whether we have a valid lock


class GPSModule:
    """
    GPS receiver interface.
    
    Provides position, velocity, and satellite information.
    Handles GPS warm-up and signal acquisition.
    """
    
    def __init__(self):
        """Initialize the GPS module."""
        self.current_position: Optional[GPSData] = None
        self.is_initialized = False
    
    def initialize(self):
        """Initialize GPS receiver and wait for first lock."""
        self.is_initialized = True
        # Placeholder for GPS hardware initialization
    
    def update(self):
        """Poll GPS and update position data."""
        self.current_position = GPSData(
            latitude=0.0,
            longitude=0.0,
            altitude=0.0,
            hdop=0.0,
            num_satellites=0,
            velocity_x=0.0,
            velocity_y=0.0,
            velocity_z=0.0,
            is_fixed=False
        )
    
    def get_position(self) -> Optional[GPSData]:
        """
        Get current GPS position and velocity.
        
        Returns:
            GPS data if available, None otherwise
        """
        return self.current_position
    
    def has_lock(self) -> bool:
        """Check if GPS has a valid position lock."""
        return (
            self.current_position is not None
            and self.current_position.is_fixed
        )
    
    def get_location_tuple(self) -> tuple[float, float, float] | None:
        """
        Get location as a tuple (latitude, longitude, altitude).
        
        Returns:
            Tuple of coordinates or None if no lock
        """
        if self.current_position and self.current_position.is_fixed:
            return (
                self.current_position.latitude,
                self.current_position.longitude,
                self.current_position.altitude,
            )
        return None
