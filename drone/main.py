"""
Main drone control entry point.

Demonstrates basic drone operations and flight loop.
"""

import time
from .flight_controller import FlightController


def main():
    """
    Main entry point for drone operation.
    
    Initializes the flight controller and runs a basic control loop.
    """
    # Create flight controller instance
    drone = FlightController()
    
    try:
        # Initialize all systems
        print("Initializing drone systems...")
        drone.initialize()
        print("Initialization complete.")
        
        # Arm the drone
        print("Arming drone...")
        if not drone.arm():
            print("Error: Failed to arm drone")
            return
        print("Drone armed.")
        
        # Takeoff
        print("Taking off to 10 meters...")
        if not drone.takeoff(target_altitude=10.0):
            print("Error: Failed to initiate takeoff")
            return
        
        # Main control loop
        dt = 0.01  # 100 Hz control rate
        flight_time = 0.0
        max_flight_time = 5.0  # seconds
        
        while flight_time < max_flight_time:
            drone.update(dt)
            flight_time += dt
            time.sleep(dt)
        
        # Landing
        print("Landing drone...")
        drone.land()
        
        # Wait for landing
        landing_time = 0.0
        max_landing_time = 10.0
        while landing_time < max_landing_time:
            drone.update(dt)
            landing_time += dt
            time.sleep(dt)
        
        # Disarm
        print("Disarming drone...")
        drone.disarm()
        
        print("Flight complete. Safe to power down.")
        
    except KeyboardInterrupt:
        print("\nInterrupt received. Landing immediately...")
        drone.land()
        drone.disarm()
    
    except Exception as e:
        print(f"Error during flight: {e}")
        drone.disarm()
    
    finally:
        # Always shutdown gracefully
        drone.shutdown()


if __name__ == "__main__":
    main()
