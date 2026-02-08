"""
Sensor Manager Module

Handles IMU, barometer, and other sensor readings.
Provides a unified interface for sensor data collection.
"""

from dataclasses import dataclass
from typing import Optional


@dataclass
class IMUData:
    """IMU (Inertial Measurement Unit) sensor data."""
    accel_x: float  # m/s^2
    accel_y: float  # m/s^2
    accel_z: float  # m/s^2
    gyro_x: float   # rad/s
    gyro_y: float   # rad/s
    gyro_z: float   # rad/s
    temperature: float  # Celsius


@dataclass
class BarometerData:
    """Barometer sensor data."""
    pressure: float  # Pa
    altitude: float  # meters
    temperature: float  # Celsius


class SensorManager:
    """
    Manages sensor data acquisition and processing.
    
    Integrates multiple sensors (IMU, barometer, compass) and provides
    clean, timestamped sensor readings.
    """
    
    def __init__(self):
        """Initialize the sensor manager."""
        self.imu_data: Optional[IMUData] = None
        self.barometer_data: Optional[BarometerData] = None
        self.compass_heading: float = 0.0  # degrees
    
    def initialize(self):
        """Initialize all sensors and perform startup calibration."""
        # Placeholder for sensor initialization
        pass
    
    def update(self):
        """Poll all sensors and update current readings."""
        # Placeholder for polling sensors
        self._read_imu()
        self._read_barometer()
        self._read_compass()
    
    def _read_imu(self):
        """Read IMU data from hardware."""
        self.imu_data = IMUData(
            accel_x=0.0,
            accel_y=0.0,
            accel_z=9.81,
            gyro_x=0.0,
            gyro_y=0.0,
            gyro_z=0.0,
            temperature=25.0
        )
    
    def _read_barometer(self):
        """Read barometer data from hardware."""
        self.barometer_data = BarometerData(
            pressure=101325.0,
            altitude=0.0,
            temperature=25.0
        )
    
    def _read_compass(self):
        """Read compass heading from hardware."""
        self.compass_heading = 0.0  # degrees
    
    def get_imu(self) -> Optional[IMUData]:
        """Get the latest IMU reading."""
        return self.imu_data
    
    def get_barometer(self) -> Optional[BarometerData]:
        """Get the latest barometer reading."""
        return self.barometer_data
    
    def get_heading(self) -> float:
        """Get the latest compass heading in degrees."""
        return self.compass_heading
