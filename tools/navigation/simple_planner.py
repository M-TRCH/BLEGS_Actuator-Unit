"""
Simple Navigation Planner for Y-axis Relative Position Control
Author: M-TRCH
Date: February 5, 2026

This module provides a simple proportional navigation planner for Y-axis movement only.
Based on the hierarchical control architecture defined in ROBOT_ARCHITECTURE.tex
"""

import numpy as np


class SimpleNavigationPlanner:
    """
    Simple Y-axis Navigation Planner (No Rotation)
    
    This planner computes velocity commands to reach a relative target position.
    It uses proportional control with velocity saturation.
    
    Mathematical Model:
        e_y = y_target - y_current
        v_y = min(v_max, K_p * |e_y|) * sign(e_y)
    
    Attributes:
        target_y (float): Target relative distance (mm)
        current_y (float): Estimated current position (mm)
        v_max (float): Maximum velocity (mm/s)
        K_p (float): Proportional gain
        tolerance (float): Position tolerance for target reached condition (mm)
    """
    
    def __init__(self, v_max: float = 50.0, K_p: float = 1.0, tolerance: float = 10.0):
        """
        Initialize the navigation planner.
        
        Args:
            v_max: Maximum velocity in mm/s (default: 50.0)
            K_p: Proportional gain (default: 1.0)
            tolerance: Position tolerance in mm (default: 10.0)
        """
        self.target_y = 0.0          # Target relative distance (mm)
        self.current_y = 0.0         # Estimated current position (mm)
        self.v_max = v_max           # Max velocity (mm/s)
        self.K_p = K_p               # Proportional gain
        self.tolerance = tolerance   # Target reached tolerance (mm)
        
        # State tracking
        self._is_active = False
        self._start_time = None
        
    def set_relative_target(self, delta_y: float) -> None:
        """
        Set relative movement target.
        
        Args:
            delta_y: Relative distance to move (mm)
                    Positive = forward, Negative = backward
        """
        self.target_y = delta_y
        self.current_y = 0.0         # Reset current position
        self._is_active = True
        print(f"  📍 Navigation target set: {delta_y:+.1f} mm")
    
    def update_position(self, position_y: float) -> None:
        """
        Update current position estimate from state estimator.
        
        Args:
            position_y: Current estimated Y position (mm)
        """
        self.current_y = position_y
    
    def compute_velocity(self) -> float:
        """
        Compute body velocity command.
        
        Returns:
            v_y: Body frame Y velocity (mm/s)
                Positive = forward, Negative = backward
        """
        if not self._is_active:
            return 0.0
        
        # Calculate position error
        e_y = self.target_y - self.current_y
        
        # Proportional control with saturation
        # v_y = min(v_max, K_p * |e_y|) * sign(e_y)
        v_magnitude = min(self.v_max, self.K_p * abs(e_y))
        v_y = v_magnitude * np.sign(e_y) if e_y != 0 else 0.0
        
        return float(v_y)
    
    def get_error(self) -> float:
        """
        Get current position error.
        
        Returns:
            Position error in mm (target - current)
        """
        return self.target_y - self.current_y
    
    def get_progress(self) -> float:
        """
        Get movement progress as percentage.
        
        Returns:
            Progress in range [0, 100]
        """
        if self.target_y == 0:
            return 100.0
        
        progress = (self.current_y / self.target_y) * 100.0
        return max(0.0, min(100.0, progress))
    
    def is_target_reached(self) -> bool:
        """
        Check if target position is reached.
        
        Condition: |e_y| < tolerance
        
        Returns:
            True if target is reached within tolerance
        """
        if not self._is_active:
            return True
        
        error = abs(self.target_y - self.current_y)
        return error < self.tolerance
    
    def stop(self) -> None:
        """Stop navigation and deactivate planner."""
        self._is_active = False
        print("  ⏹️  Navigation stopped")
    
    def reset(self) -> None:
        """Reset planner to initial state."""
        self.target_y = 0.0
        self.current_y = 0.0
        self._is_active = False
        self._start_time = None
    
    def get_status(self) -> dict:
        """
        Get current planner status.
        
        Returns:
            Dictionary with status information
        """
        return {
            'active': self._is_active,
            'target_y': self.target_y,
            'current_y': self.current_y,
            'error': self.get_error(),
            'progress': self.get_progress(),
            'target_reached': self.is_target_reached(),
            'v_max': self.v_max,
            'K_p': self.K_p,
            'tolerance': self.tolerance
        }
    
    def __repr__(self) -> str:
        return (f"SimpleNavigationPlanner(v_max={self.v_max}, K_p={self.K_p}, "
                f"tolerance={self.tolerance}, active={self._is_active})")
