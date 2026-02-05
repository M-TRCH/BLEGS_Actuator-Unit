"""
Time-based State Estimator for Relative Position Control
Author: M-TRCH
Date: February 5, 2026

This module provides a simple time-based position estimator using dead reckoning.
Based on the hierarchical control architecture defined in ROBOT_ARCHITECTURE.tex
"""

import time


class TimeBasedEstimator:
    """
    Simple Time-based Position Estimator (Dead Reckoning)
    
    This estimator computes position by integrating velocity over time.
    It's a simplified version suitable for testing the hierarchical architecture.
    
    Mathematical Model:
        y_{k+1} = y_k + v_{y,k} * Δt
    
    Limitations:
        - Drift error accumulates over time
        - No sensor feedback (slip detection)
        - Accuracy decreases with distance
        - Recommended for distances < 1m
    
    Attributes:
        position_y (float): Estimated Y position (mm)
        velocity_y (float): Current velocity (mm/s)
        total_distance (float): Total distance traveled (mm)
    """
    
    def __init__(self):
        """Initialize the time-based estimator."""
        self.position_y = 0.0           # Estimated Y position (mm)
        self.velocity_y = 0.0           # Current velocity (mm/s)
        self.total_distance = 0.0       # Total distance traveled (mm)
        self._last_update_time = None   # Last update timestamp
        self._start_time = None         # Movement start time
        self._update_count = 0          # Number of updates
        
    def start(self) -> None:
        """Start the estimator (reset and begin timing)."""
        self.reset()
        self._start_time = time.time()
        self._last_update_time = self._start_time
        print("  📏 State estimator started")
    
    def update(self, v_body_y: float, current_time: float = None) -> None:
        """
        Update position estimate using velocity and time.
        
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
        return {
            'position_y': self.position_y,
            'velocity_y': self.velocity_y,
            'total_distance': self.total_distance,
            'elapsed_time': self.get_elapsed_time(),
            'average_velocity': self.get_average_velocity(),
            'update_count': self._update_count
        }
    
    def __repr__(self) -> str:
        return (f"TimeBasedEstimator(position_y={self.position_y:.2f}mm, "
                f"velocity_y={self.velocity_y:.2f}mm/s)")
