"""
Motor Feedback Log Analyzer & Plotter
Author: M-TRCH
Date: December 30, 2025

This script reads motor feedback CSV logs and creates comprehensive visualizations:
- Position tracking (setpoint vs actual)
- Position error analysis
- Current consumption
- Status flags monitoring

Usage:
    python Plot_Motor_Log.py [log_file.csv]
    
If no file specified, will look for latest log in logs/ directory
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys
import os
from pathlib import Path

# ============================================================================
# CONFIGURATION
# ============================================================================

LOG_DIRECTORY = "logs"
MOTOR_NAMES = {
    1: 'FL-A (Front Left Inner)',
    2: 'FL-B (Front Left Outer)',
    3: 'FR-A (Front Right Inner)',
    4: 'FR-B (Front Right Outer)',
    5: 'RL-A (Rear Left Inner)',
    6: 'RL-B (Rear Left Outer)',
    7: 'RR-A (Rear Right Inner)',
    8: 'RR-B (Rear Right Outer)'
}

LEG_COLORS = {
    'FL': '#1f77b4',  # Blue
    'FR': '#ff7f0e',  # Orange
    'RL': '#2ca02c',  # Green
    'RR': '#d62728'   # Red
}

# ============================================================================
# HELPER FUNCTIONS
# ============================================================================

def find_latest_log():
    """Find the most recent log file in logs directory"""
    log_dir = Path(LOG_DIRECTORY)
    if not log_dir.exists():
        return None
    
    csv_files = list(log_dir.glob('*.csv'))
    if not csv_files:
        return None
    
    # Sort by modification time, return latest
    latest = max(csv_files, key=lambda p: p.stat().st_mtime)
    return str(latest)

def get_motor_leg_info(motor_id):
    """Get leg ID and motor type (A/B) from motor ID"""
    leg_map = {
        1: ('FL', 'A'), 2: ('FL', 'B'),
        3: ('FR', 'A'), 4: ('FR', 'B'),
        5: ('RL', 'A'), 6: ('RL', 'B'),
        7: ('RR', 'A'), 8: ('RR', 'B')
    }
    return leg_map.get(motor_id, ('??', '?'))

def load_log_data(filepath):
    """Load and parse CSV log file"""
    try:
        df = pd.read_csv(filepath)
        print(f"✅ Loaded: {filepath}")
        print(f"   Records: {len(df)}")
        print(f"   Duration: {df['elapsed_ms'].max() / 1000:.1f} seconds")
        print(f"   Motors: {sorted(df['motor_id'].unique())}")
        return df
    except Exception as e:
        print(f"❌ Error loading file: {e}")
        return None

# ============================================================================
# PLOTTING FUNCTIONS
# ============================================================================

def plot_position_tracking(df, motor_ids=None):
    """Plot position setpoint vs actual for each motor"""
    if motor_ids is None:
        motor_ids = sorted(df['motor_id'].unique())
    
    num_motors = len(motor_ids)
    fig, axes = plt.subplots(num_motors, 1, figsize=(14, 2.5*num_motors), sharex=True)
    
    if num_motors == 1:
        axes = [axes]
    
    fig.suptitle('Motor Position Tracking', fontsize=16, weight='bold')
    
    for idx, motor_id in enumerate(motor_ids):
        ax = axes[idx]
        motor_data = df[df['motor_id'] == motor_id]
        
        time_s = motor_data['elapsed_ms'] / 1000.0
        
        # Get leg info for color
        leg_id, motor_type = get_motor_leg_info(motor_id)
        color = LEG_COLORS.get(leg_id, 'gray')
        
        # Plot setpoint and actual
        ax.plot(time_s, motor_data['setpoint_deg'], '--', 
                color=color, alpha=0.7, linewidth=1.5, label='Setpoint')
        ax.plot(time_s, motor_data['position_deg'], '-', 
                color=color, linewidth=2, label='Actual')
        
        ax.set_ylabel('Position (°)', fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right', fontsize=9)
        
        motor_name = MOTOR_NAMES.get(motor_id, f'Motor {motor_id}')
        ax.set_title(f'{motor_name}', fontsize=10, weight='bold')
        
        # Calculate tracking statistics
        mean_error = motor_data['error_deg'].mean()
        max_error = motor_data['error_deg'].abs().max()
        rms_error = np.sqrt((motor_data['error_deg']**2).mean())
        
        stats_text = f'Mean Err: {mean_error:+.2f}° | Max: {max_error:.2f}° | RMS: {rms_error:.2f}°'
        ax.text(0.02, 0.05, stats_text, transform=ax.transAxes,
                fontsize=8, verticalalignment='bottom',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    axes[-1].set_xlabel('Time (s)', fontsize=11)
    plt.tight_layout()
    return fig

def plot_position_error(df, motor_ids=None):
    """Plot position error for each motor"""
    if motor_ids is None:
        motor_ids = sorted(df['motor_id'].unique())
    
    num_motors = len(motor_ids)
    fig, axes = plt.subplots(num_motors, 1, figsize=(14, 2*num_motors), sharex=True)
    
    if num_motors == 1:
        axes = [axes]
    
    fig.suptitle('Position Error Analysis', fontsize=16, weight='bold')
    
    for idx, motor_id in enumerate(motor_ids):
        ax = axes[idx]
        motor_data = df[df['motor_id'] == motor_id]
        
        time_s = motor_data['elapsed_ms'] / 1000.0
        
        leg_id, motor_type = get_motor_leg_info(motor_id)
        color = LEG_COLORS.get(leg_id, 'gray')
        
        ax.plot(time_s, motor_data['error_deg'], '-', 
                color=color, linewidth=1.5, alpha=0.8)
        ax.axhline(y=0, color='black', linestyle='--', linewidth=0.8, alpha=0.5)
        
        ax.set_ylabel('Error (°)', fontsize=10)
        ax.grid(True, alpha=0.3)
        
        motor_name = MOTOR_NAMES.get(motor_id, f'Motor {motor_id}')
        ax.set_title(f'{motor_name}', fontsize=10, weight='bold')
        
        # Statistics
        std_error = motor_data['error_deg'].std()
        ax.fill_between(time_s, -std_error, std_error, 
                        alpha=0.2, color=color, label=f'±1σ ({std_error:.2f}°)')
        ax.legend(loc='upper right', fontsize=8)
    
    axes[-1].set_xlabel('Time (s)', fontsize=11)
    plt.tight_layout()
    return fig

def plot_current_consumption(df, motor_ids=None):
    """Plot current consumption for each motor"""
    if motor_ids is None:
        motor_ids = sorted(df['motor_id'].unique())
    
    num_motors = len(motor_ids)
    fig, axes = plt.subplots(num_motors, 1, figsize=(14, 2*num_motors), sharex=True)
    
    if num_motors == 1:
        axes = [axes]
    
    fig.suptitle('Motor Current Consumption', fontsize=16, weight='bold')
    
    for idx, motor_id in enumerate(motor_ids):
        ax = axes[idx]
        motor_data = df[df['motor_id'] == motor_id]
        
        time_s = motor_data['elapsed_ms'] / 1000.0
        
        leg_id, motor_type = get_motor_leg_info(motor_id)
        color = LEG_COLORS.get(leg_id, 'gray')
        
        ax.plot(time_s, motor_data['current_mA'], '-', 
                color=color, linewidth=1.5, alpha=0.8)
        
        ax.set_ylabel('Current (mA)', fontsize=10)
        ax.grid(True, alpha=0.3)
        
        motor_name = MOTOR_NAMES.get(motor_id, f'Motor {motor_id}')
        ax.set_title(f'{motor_name}', fontsize=10, weight='bold')
        
        # Statistics
        mean_current = motor_data['current_mA'].mean()
        max_current = motor_data['current_mA'].max()
        
        stats_text = f'Mean: {mean_current:.0f} mA | Peak: {max_current:.0f} mA'
        ax.text(0.02, 0.95, stats_text, transform=ax.transAxes,
                fontsize=8, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
    
    axes[-1].set_xlabel('Time (s)', fontsize=11)
    plt.tight_layout()
    return fig

def plot_summary_statistics(df):
    """Plot summary statistics for all motors"""
    motor_ids = sorted(df['motor_id'].unique())
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Summary Statistics - All Motors', fontsize=16, weight='bold')
    
    # Prepare data
    stats_data = []
    for motor_id in motor_ids:
        motor_data = df[df['motor_id'] == motor_id]
        leg_id, motor_type = get_motor_leg_info(motor_id)
        
        stats_data.append({
            'motor_id': motor_id,
            'leg': leg_id,
            'type': motor_type,
            'mean_error': motor_data['error_deg'].mean(),
            'rms_error': np.sqrt((motor_data['error_deg']**2).mean()),
            'max_error': motor_data['error_deg'].abs().max(),
            'mean_current': motor_data['current_mA'].mean(),
            'peak_current': motor_data['current_mA'].max()
        })
    
    stats_df = pd.DataFrame(stats_data)
    
    # Plot 1: RMS Error by Motor
    ax1 = axes[0, 0]
    colors = [LEG_COLORS.get(leg, 'gray') for leg in stats_df['leg']]
    bars1 = ax1.bar(stats_df['motor_id'], stats_df['rms_error'], color=colors, alpha=0.7, edgecolor='black')
    ax1.set_xlabel('Motor ID', fontsize=11)
    ax1.set_ylabel('RMS Error (°)', fontsize=11)
    ax1.set_title('Position RMS Error', fontsize=12, weight='bold')
    ax1.grid(True, alpha=0.3, axis='y')
    ax1.set_xticks(motor_ids)
    
    # Add value labels
    for bar in bars1:
        height = bar.get_height()
        ax1.text(bar.get_x() + bar.get_width()/2., height,
                f'{height:.2f}°', ha='center', va='bottom', fontsize=9)
    
    # Plot 2: Max Error by Motor
    ax2 = axes[0, 1]
    bars2 = ax2.bar(stats_df['motor_id'], stats_df['max_error'], color=colors, alpha=0.7, edgecolor='black')
    ax2.set_xlabel('Motor ID', fontsize=11)
    ax2.set_ylabel('Max Error (°)', fontsize=11)
    ax2.set_title('Maximum Position Error', fontsize=12, weight='bold')
    ax2.grid(True, alpha=0.3, axis='y')
    ax2.set_xticks(motor_ids)
    
    for bar in bars2:
        height = bar.get_height()
        ax2.text(bar.get_x() + bar.get_width()/2., height,
                f'{height:.2f}°', ha='center', va='bottom', fontsize=9)
    
    # Plot 3: Mean Current by Motor
    ax3 = axes[1, 0]
    bars3 = ax3.bar(stats_df['motor_id'], stats_df['mean_current'], color=colors, alpha=0.7, edgecolor='black')
    ax3.set_xlabel('Motor ID', fontsize=11)
    ax3.set_ylabel('Mean Current (mA)', fontsize=11)
    ax3.set_title('Average Current Consumption', fontsize=12, weight='bold')
    ax3.grid(True, alpha=0.3, axis='y')
    ax3.set_xticks(motor_ids)
    
    for bar in bars3:
        height = bar.get_height()
        ax3.text(bar.get_x() + bar.get_width()/2., height,
                f'{height:.0f}', ha='center', va='bottom', fontsize=9)
    
    # Plot 4: Peak Current by Motor
    ax4 = axes[1, 1]
    bars4 = ax4.bar(stats_df['motor_id'], stats_df['peak_current'], color=colors, alpha=0.7, edgecolor='black')
    ax4.set_xlabel('Motor ID', fontsize=11)
    ax4.set_ylabel('Peak Current (mA)', fontsize=11)
    ax4.set_title('Peak Current Consumption', fontsize=12, weight='bold')
    ax4.grid(True, alpha=0.3, axis='y')
    ax4.set_xticks(motor_ids)
    
    for bar in bars4:
        height = bar.get_height()
        ax4.text(bar.get_x() + bar.get_width()/2., height,
                f'{height:.0f}', ha='center', va='bottom', fontsize=9)
    
    plt.tight_layout()
    return fig, stats_df

def plot_error_distribution(df, motor_ids=None):
    """Plot error distribution histograms"""
    if motor_ids is None:
        motor_ids = sorted(df['motor_id'].unique())
    
    num_motors = len(motor_ids)
    cols = 4
    rows = (num_motors + cols - 1) // cols
    
    fig, axes = plt.subplots(rows, cols, figsize=(16, 3*rows))
    fig.suptitle('Position Error Distribution', fontsize=16, weight='bold')
    
    axes = axes.flatten() if num_motors > 1 else [axes]
    
    for idx, motor_id in enumerate(motor_ids):
        ax = axes[idx]
        motor_data = df[df['motor_id'] == motor_id]
        
        leg_id, motor_type = get_motor_leg_info(motor_id)
        color = LEG_COLORS.get(leg_id, 'gray')
        
        # Histogram
        ax.hist(motor_data['error_deg'], bins=50, color=color, alpha=0.7, edgecolor='black')
        
        # Mean and std lines
        mean_err = motor_data['error_deg'].mean()
        std_err = motor_data['error_deg'].std()
        
        ax.axvline(mean_err, color='red', linestyle='--', linewidth=2, label=f'Mean: {mean_err:.2f}°')
        ax.axvline(mean_err - std_err, color='orange', linestyle=':', linewidth=1.5, alpha=0.7)
        ax.axvline(mean_err + std_err, color='orange', linestyle=':', linewidth=1.5, alpha=0.7, label=f'±σ: {std_err:.2f}°')
        
        motor_name = MOTOR_NAMES.get(motor_id, f'Motor {motor_id}')
        ax.set_title(f'{motor_name}', fontsize=10, weight='bold')
        ax.set_xlabel('Error (°)', fontsize=9)
        ax.set_ylabel('Frequency', fontsize=9)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3, axis='y')
    
    # Hide unused subplots
    for idx in range(num_motors, len(axes)):
        axes[idx].set_visible(False)
    
    plt.tight_layout()
    return fig

# ============================================================================
# MAIN FUNCTION
# ============================================================================

def main():
    """Main execution function"""
    print("\n" + "="*70)
    print("  📊 Motor Feedback Log Analyzer")
    print("="*70)
    
    # Determine log file to analyze
    if len(sys.argv) > 1:
        log_file = sys.argv[1]
    else:
        log_file = find_latest_log()
        if log_file is None:
            print(f"\n❌ No log files found in '{LOG_DIRECTORY}/' directory")
            print("Usage: python Plot_Motor_Log.py [log_file.csv]")
            return
        print(f"\n📁 Using latest log file")
    
    # Load data
    df = load_log_data(log_file)
    if df is None:
        return
    
    print("\n📈 Generating plots...")
    
    # Create plots
    fig1 = plot_position_tracking(df)
    fig2 = plot_position_error(df)
    fig3 = plot_current_consumption(df)
    fig4, stats_df = plot_summary_statistics(df)
    fig5 = plot_error_distribution(df)
    
    # Print summary statistics
    print("\n" + "="*70)
    print("  📊 SUMMARY STATISTICS")
    print("="*70)
    print(stats_df.to_string(index=False))
    print("="*70)
    
    # Show all plots
    print("\n✅ All plots generated successfully!")
    print("   Close plot windows to exit...")
    plt.show()

if __name__ == "__main__":
    main()
