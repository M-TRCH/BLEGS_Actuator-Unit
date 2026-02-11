"""
Time-based State Estimator for Relative Position Control
Author: M-TRCH
Date: February 5, 2026 (Updated: February 11, 2026)

This module provides a simple time-based position estimator using dead reckoning.
Based on the hierarchical control architecture defined in ROBOT_ARCHITECTURE.tex

Update (Feb 11, 2026): Added IMU integration for yaw tracking
"""

import time
from typing import Optional


class YawController:
    """
    PD Controller for yaw orientation correction.
    
    This controller computes a lateral offset to correct yaw drift
    by adjusting the gait trajectory based on IMU feedback.
    
    Mathematical Model:
        u_yaw = K_p * e_yaw + K_d * (de_yaw/dt)
        
    where:
        e_yaw = yaw_target - yaw_current (in degrees)
        u_yaw = lateral offset command (mm)
    
    Attributes:
        K_p (float): Proportional gain
        K_d (float): Derivative gain
        target_yaw (float): Target yaw angle (degrees)
    """
    
    def __init__(self, K_p: float = 0.5, K_d: float = 0.1, max_correction: float = 10.0):
        """
        Initialize yaw controller.
        
        Args:
            K_p: Proportional gain (default: 0.5 mm/deg)
            K_d: Derivative gain (default: 0.1 mm/(deg/s))
            max_correction: Maximum lateral correction (mm)
        """
        self.K_p = K_p
        self.K_d = K_d
        self.max_correction = max_correction
        
        # Controller state
        self.target_yaw = 0.0
        self._last_error = 0.0
        self._last_update_time = None
        
    def set_target(self, yaw_deg: float) -> None:
        """
        Set target yaw angle.
        
        Args:
            yaw_deg: Target yaw in degrees
        """
        self.target_yaw = yaw_deg
        
    def reset(self) -> None:
        """Reset controller state."""
        self.target_yaw = 0.0
        self._last_error = 0.0
        self._last_update_time = None
    
    def compute(self, current_yaw: float, current_time: float = None) -> float:
        """
        Compute yaw correction command.
        
        Convention (Differential Step Forward): 
            - Robot drifted left (-yaw) → Positive correction → Turn right to compensate
            - Robot drifted right (+yaw) → Negative correction → Turn left to compensate
        
        Args:
            current_yaw: Current yaw from IMU (degrees)
                        Left turn = negative (-), Right turn = positive (+)
            current_time: Current time in seconds
        
        Returns:
            Differential step forward correction in mm:
                Positive = turn right (left legs step MORE forward)
                Negative = turn left (right legs step MORE forward)
        """
        if current_time is None:
            current_time = time.time()
        
        # Compute error (target - current)
        # If robot drifted left (-yaw): error = positive → turn right
        # If robot drifted right (+yaw): error = negative → turn left
        error = self.target_yaw - current_yaw
        
        # Proportional term
        p_term = self.K_p * error
        
        # Derivative term
        d_term = 0.0
        if self._last_update_time is not None:
            dt = current_time - self._last_update_time
            if dt > 0:
                d_error = (error - self._last_error) / dt
                d_term = self.K_d * d_error
        
        # Compute correction
        correction = p_term + d_term
        
        # Apply limits
        correction = max(-self.max_correction, min(self.max_correction, correction))
        
        # Update state
        self._last_error = error
        self._last_update_time = current_time
        
        return correction


class TimeBasedEstimator:
    """
    Simple Time-based Position Estimator (Dead Reckoning) with IMU Integration
    
    This estimator computes position by integrating velocity over time.
    Optionally integrates IMU for yaw orientation tracking.
    
    Mathematical Model:
        y_{k+1} = y_k + v_{y,k} * Δt
        yaw_{k} = from IMU (if available)
    
    Limitations:
        - Drift error accumulates over time (without IMU)
        - No sensor feedback for slip detection
        - Accuracy decreases with distance
        - Recommended for distances < 1m (without IMU)
    
    Attributes:
        position_y (float): Estimated Y position (mm)
        velocity_y (float): Current velocity (mm/s)
        total_distance (float): Total distance traveled (mm)
        yaw (float): Current yaw orientation (degrees, from IMU)
    """
    
    def __init__(self, imu_reader=None):
        """
        Initialize the time-based estimator.
        
        Args:
            imu_reader: Optional IMUReader instance for yaw tracking
        """
        self.position_y = 0.0           # Estimated Y position (mm)
        self.velocity_y = 0.0           # Current velocity (mm/s)
        self.total_distance = 0.0       # Total distance traveled (mm)
        self._last_update_time = None   # Last update timestamp
        self._start_time = None         # Movement start time
        self._update_count = 0          # Number of updates
        
        # IMU integration
        self._imu_reader = imu_reader
        self.yaw = 0.0                  # Current yaw (degrees)
        self.target_yaw = 0.0           # Target yaw (degrees)
        self._imu_available = (imu_reader is not None)
        
    def start(self) -> None:
        """Start the estimator (reset and begin timing)."""
        self.reset()
        self._start_time = time.time()
        self._last_update_time = self._start_time
        
        # Set target yaw from IMU if available
        if self._imu_available and self._imu_reader.is_connected():
            self.target_yaw = self._imu_reader.get_yaw()
            print(f"  📏 State estimator started (IMU yaw: {self.target_yaw:.1f}°)")
        else:
            print("  📏 State estimator started (no IMU)")
    
    def update(self, v_body_y: float, current_time: float = None) -> None:
        """
        Update position estimate using velocity and time.
        Also updates yaw from IMU if available.
        
        Uses dead reckoning: y_{k+1} = y_k + v_y * Δt
        
        Args:
            v_body_y: Body frame Y velocity (mm/s)
            current_time: Current time in seconds (uses time.time() if None)
        """
        if current_time is None:
            current_time = time.time()
        
        if self._last_update_time is not None:
            dt = current_time - self._last_update_time
            
            # Dead reckoning position update
            displacement = v_body_y * dt
            self.position_y += displacement
            self.total_distance += abs(displacement)
        
        # Update yaw from IMU if available
        if self._imu_available and self._imu_reader.is_connected():
            self.yaw = self._imu_reader.get_yaw()
            
        self.velocity_y = v_body_y
        self._last_update_time = current_time
        self._update_count += 1
    
    def reset(self) -> None:
        """Reset position estimate to zero (for new movement command)."""
        self.position_y = 0.0
        self.velocity_y = 0.0
        self.total_distance = 0.0
        self._last_update_time = None
        self._start_time = None
        self._update_count = 0
        
        # Reset yaw target
        if self._imu_available and self._imu_reader.is_connected():
            self.target_yaw = self._imu_reader.get_yaw()
            self.yaw = self.target_yaw
    
    def get_position(self) -> float:
        """
        Get estimated Y position.
        
        Returns:
            Estimated Y position in mm
        """
        return self.position_y
    
    def get_velocity(self) -> float:
        """
        Get current velocity.
        
        Returns:
            Current velocity in mm/s
        """
        return self.velocity_y
    
    def get_elapsed_time(self) -> float:
        """
        Get elapsed time since start.
        
        Returns:
            Elapsed time in seconds, or 0 if not started
        """
        if self._start_time is None:
            return 0.0
        return time.time() - self._start_time
    
    def get_average_velocity(self) -> float:
        """
        Get average velocity since start.
        
        Returns:
            Average velocity in mm/s
        """
        elapsed = self.get_elapsed_time()
        if elapsed <= 0:
            return 0.0
        return self.position_y / elapsed
    
    def get_status(self) -> dict:
        """
        Get current estimator status.
        
        Returns:
            Dictionary with status information
        """
        status = {
            'position_y': self.position_y,
            'velocity_y': self.velocity_y,
            'total_distance': self.total_distance,
            'elapsed_time': self.get_elapsed_time(),
            'average_velocity': self.get_average_velocity(),
            'update_count': self._update_count
        }
        
        # Add IMU status if available
        if self._imu_available:
            status['yaw'] = self.yaw
            status['target_yaw'] = self.target_yaw
            status['yaw_error'] = self.get_yaw_error()
            status['imu_connected'] = self._imu_reader.is_connected()
            status['imu_calibrated'] = self._imu_reader.is_calibrated()
        
        return status
    
    def get_yaw(self) -> float:
        """
        Get current yaw angle.
        
        Returns:
            Current yaw in degrees
        """
        return self.yaw
    
    def get_yaw_error(self) -> float:
        """
        Get yaw error (target - current).
        
        Returns:
            Yaw error in degrees
        """
        return self.target_yaw - self.yaw
    
    def has_imu(self) -> bool:
        """Check if IMU is available and connected."""
        return self._imu_available and (self._imu_reader.is_connected() if self._imu_reader else False)
    
    def __repr__(self) -> str:
        if self._imu_available:
            return (f"TimeBasedEstimator(position_y={self.position_y:.2f}mm, "
                    f"velocity_y={self.velocity_y:.2f}mm/s, yaw={self.yaw:.1f}°)")
        else:
            return (f"TimeBasedEstimator(position_y={self.position_y:.2f}mm, "
                    f"velocity_y={self.velocity_y:.2f}mm/s)")
