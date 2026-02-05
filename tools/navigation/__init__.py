"""
Navigation Module for Relative Position Control
Author: M-TRCH
Date: February 5, 2026

This module provides simple navigation components for quadruped robot control.
"""

from .simple_planner import SimpleNavigationPlanner
from .time_estimator import TimeBasedEstimator

__all__ = ['SimpleNavigationPlanner', 'TimeBasedEstimator']
