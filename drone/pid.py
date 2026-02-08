"""
PID Controller Implementation

Provides a generic PID controller for stabilization and navigation control.
"""


class PIDController:
    """
    A Proportional-Integral-Derivative controller for control systems.
    
    Used for stabilizing attitude, altitude, and position of the drone.
    """
    
    def __init__(self, kp: float, ki: float, kd: float, setpoint: float = 0.0):
        """
        Initialize PID controller with gain coefficients.
        
        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            setpoint: Target value for the controlled variable
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None
    
    def update(self, current_value: float, dt: float) -> float:
        """
        Calculate the control output based on current value.
        
        Args:
            current_value: Current measured value
            dt: Time step in seconds
            
        Returns:
            Control output value
        """
        error = self.setpoint - current_value
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # Derivative term
        d_term = self.kd * (error - self.last_error) / dt if dt > 0 else 0.0
        
        self.last_error = error
        
        return p_term + i_term + d_term
    
    def reset(self):
        """Reset the controller state."""
        self.integral = 0.0
        self.last_error = 0.0
    
    def set_setpoint(self, setpoint: float):
        """Set the target value."""
        self.setpoint = setpoint
