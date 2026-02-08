"""
Autonomous Drone Control System

A modular Python package for controlling autonomous quadcopter drones.
Provides flight control, sensor integration, and telemetry tracking.
"""

__version__ = "0.1.0"
__author__ = "Drone Development Team"

from .flight_controller import FlightController
from .sensors import SensorManager
from .telemetry import TelemetryManager
from .gps import GPSModule

__all__ = [
    "FlightController",
    "SensorManager",
    "TelemetryManager",
    "GPSModule",
]
