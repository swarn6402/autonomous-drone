"""
PID Controller Implementation

Provides a generic PID controller for stabilization and navigation control.
Includes anti-windup protection for the integral term.
"""


class PIDController:
    """
    A Proportional-Integral-Derivative controller for control systems.
    
    Implements a standard PID control law with anti-windup protection
    for the integral term. Used for stabilizing attitude, altitude, and
    position of the drone.
    """
    
    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        setpoint: float = 0.0,
        integral_limit: float = 100.0,
        output_limit: float = None,
    ):
        """
        Initialize PID controller with gain coefficients.
        
        Args:
            kp: Proportional gain coefficient
            ki: Integral gain coefficient
            kd: Derivative gain coefficient
            setpoint: Target/desired value for the controlled variable
            integral_limit: Maximum magnitude for integral accumulation (anti-windup)
            output_limit: Maximum magnitude for controller output (optional clamping)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        
        # Internal state
        self.integral = 0.0  # Accumulated integral error
        self.last_error = 0.0  # Previous error for derivative calculation
    
    def update(self, measured_value: float, dt: float) -> float:
        """
        Calculate the control output based on measured value.
        
        Implements the discrete-time PID control law:
        output = kp*e(t) + ki*∫e(t)dt + kd*de(t)/dt
        
        Args:
            measured_value: Current measured/feedback value
            dt: Time step in seconds
            
        Returns:
            Control output value (clamped if output_limit is set)
        """
        # Calculate error signal
        error = self.setpoint - measured_value
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term with anti-windup clamping
        self.integral += error * dt
        # Clamp integral to prevent windup
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        i_term = self.ki * self.integral
        
        # Derivative term (using backward difference)
        if dt > 0:
            d_term = self.kd * (error - self.last_error) / dt
        else:
            d_term = 0.0
        
        # Update state for next iteration
        self.last_error = error
        
        # Calculate total output
        output = p_term + i_term + d_term
        
        # Optional: clamp output to limits
        if self.output_limit is not None:
            output = max(-self.output_limit, min(self.output_limit, output))
        
        return output
    
    def reset(self):
        """Reset the controller state."""
        self.integral = 0.0
        self.last_error = 0.0
    
    def set_setpoint(self, setpoint: float):
        """
        Set a new target/setpoint value.
        
        Args:
            setpoint: New target value
        """
        self.setpoint = setpoint
    
    def set_gains(self, kp: float, ki: float, kd: float):
        """
        Update controller gain coefficients.
        
        Args:
            kp: New proportional gain
            ki: New integral gain
            kd: New derivative gain
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
