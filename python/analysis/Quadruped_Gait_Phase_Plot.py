"""
Quadruped Gait Phase Visualization
===================================
Plot phase diagrams for different quadruped gaits (8-DOF robot without Hip Roll)

Gaits: Crawl, Trot, Pace, Bound, Pronk, Gallop

Leg Convention:
    LF - Left Front
    RF - Right Front  
    LH - Left Hind
    RH - Right Hind

Phase: 0 to 1 represents one full gait cycle
       Swing phase shown as colored bars
       Stance phase shown as white/empty
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import Rectangle

# Define leg names
LEGS = ['LF', 'RF', 'LH', 'RH']
LEG_COLORS = {
    'LF': '#E74C3C',  # Red
    'RF': '#3498DB',  # Blue
    'LH': '#2ECC71',  # Green
    'RH': '#9B59B6'   # Purple
}

# Gait definitions: (phase_offset, duty_factor)
# phase_offset: when the swing phase starts (0-1)
# duty_factor: fraction of cycle in stance phase

GAITS = {
    'Crawl': {
        'description': 'Walking gait - one leg at a time',
        'phases': {'LF': 0.0, 'RH': 0.25, 'RF': 0.5, 'LH': 0.75},
        'duty_factor': 0.75,  # 75% stance, 25% swing
    },
    'Trot': {
        'description': 'Diagonal legs move together',
        'phases': {'LF': 0.0, 'RH': 0.0, 'RF': 0.5, 'LH': 0.5},
        'duty_factor': 0.5,  # 50% stance, 50% swing
    },
    'Pace': {
        'description': 'Same-side legs move together',
        'phases': {'LF': 0.0, 'LH': 0.0, 'RF': 0.5, 'RH': 0.5},
        'duty_factor': 0.5,
    },
    'Bound': {
        'description': 'Front legs together, hind legs together',
        'phases': {'LF': 0.0, 'RF': 0.0, 'LH': 0.5, 'RH': 0.5},
        'duty_factor': 0.4,
    },
    'Pronk': {
        'description': 'All four legs move together (hopping)',
        'phases': {'LF': 0.0, 'RF': 0.0, 'LH': 0.0, 'RH': 0.0},
        'duty_factor': 0.3,
    },
    'Gallop': {
        'description': 'Asymmetric running gait',
        'phases': {'LF': 0.0, 'RF': 0.1, 'LH': 0.5, 'RH': 0.6},
        'duty_factor': 0.35,
    },
}


def plot_gait_phase_diagram(gait_name, gait_data, ax, num_cycles=2):
    """
    Plot phase diagram for a single gait
    
    Parameters:
    -----------
    gait_name : str
        Name of the gait
    gait_data : dict
        Dictionary containing phases and duty_factor
    ax : matplotlib axis
        Axis to plot on
    num_cycles : int
        Number of gait cycles to display
    """
    phases = gait_data['phases']
    duty_factor = gait_data['duty_factor']
    swing_duration = 1 - duty_factor
    
    y_positions = {leg: i for i, leg in enumerate(LEGS)}
    
    for leg in LEGS:
        phase_offset = phases[leg]
        y = y_positions[leg]
        color = LEG_COLORS[leg]
        
        # Plot multiple cycles
        for cycle in range(num_cycles + 1):
            # Swing phase starts at phase_offset
            swing_start = cycle + phase_offset
            swing_end = swing_start + swing_duration
            
            # Only draw if within display range
            if swing_start < num_cycles:
                rect = Rectangle(
                    (swing_start, y - 0.35),
                    min(swing_duration, num_cycles - swing_start),
                    0.7,
                    facecolor=color,
                    edgecolor='black',
                    linewidth=1.5,
                    alpha=0.8
                )
                ax.add_patch(rect)
            
            # Handle wrap-around for swing phase
            if phase_offset + swing_duration > 1 and cycle < num_cycles:
                wrap_start = cycle
                wrap_duration = (phase_offset + swing_duration) - 1
                if cycle > 0 or phase_offset + swing_duration > 1:
                    rect_wrap = Rectangle(
                        (wrap_start, y - 0.35),
                        min(wrap_duration, num_cycles - wrap_start) if cycle == 0 else 0,
                        0.7,
                        facecolor=color,
                        edgecolor='black',
                        linewidth=1.5,
                        alpha=0.8
                    )
                    if wrap_duration > 0 and cycle > 0:
                        ax.add_patch(rect_wrap)
    
    # Formatting
    ax.set_xlim(0, num_cycles)
    ax.set_ylim(-0.5, len(LEGS) - 0.5)
    ax.set_yticks(range(len(LEGS)))
    ax.set_yticklabels(LEGS, fontsize=11, fontweight='bold')
    ax.set_xlabel('Gait Cycle', fontsize=10)
    ax.set_title(f'{gait_name}\n{gait_data["description"]}', fontsize=12, fontweight='bold')
    
    # Add vertical lines for cycle boundaries
    for i in range(num_cycles + 1):
        ax.axvline(x=i, color='gray', linestyle='--', linewidth=0.8, alpha=0.7)
    
    # Add horizontal grid
    for i in range(len(LEGS)):
        ax.axhline(y=i - 0.5, color='lightgray', linestyle='-', linewidth=0.5)
    ax.axhline(y=len(LEGS) - 0.5, color='lightgray', linestyle='-', linewidth=0.5)
    
    ax.set_aspect('equal')
    ax.grid(False)


def plot_all_gaits():
    """Create a figure with all gait phase diagrams"""
    fig, axes = plt.subplots(2, 3, figsize=(16, 10))
    fig.suptitle('Quadruped Gait Phase Diagrams (8-DOF Robot)\nSwing Phase Visualization', 
                 fontsize=14, fontweight='bold', y=0.98)
    
    axes_flat = axes.flatten()
    
    for idx, (gait_name, gait_data) in enumerate(GAITS.items()):
        plot_gait_phase_diagram(gait_name, gait_data, axes_flat[idx])
    
    # Add legend
    legend_patches = [mpatches.Patch(color=LEG_COLORS[leg], label=leg) for leg in LEGS]
    fig.legend(handles=legend_patches, loc='lower center', ncol=4, fontsize=11,
               frameon=True, fancybox=True, shadow=True, 
               bbox_to_anchor=(0.5, 0.02))
    
    plt.tight_layout(rect=[0, 0.06, 1, 0.95])
    return fig


def plot_gait_timing_bars():
    """Create horizontal bar chart showing leg timing for each gait"""
    fig, axes = plt.subplots(2, 3, figsize=(16, 10))
    fig.suptitle('Quadruped Gait Timing Diagrams\n(Colored = Swing, White = Stance)', 
                 fontsize=14, fontweight='bold', y=0.98)
    
    axes_flat = axes.flatten()
    
    for idx, (gait_name, gait_data) in enumerate(GAITS.items()):
        ax = axes_flat[idx]
        phases = gait_data['phases']
        duty_factor = gait_data['duty_factor']
        swing_duration = 1 - duty_factor
        
        y_positions = list(range(len(LEGS)))
        
        # Draw background (stance phase)
        for i, leg in enumerate(LEGS):
            ax.barh(i, 1.0, height=0.6, color='white', edgecolor='black', linewidth=1)
        
        # Draw swing phases
        for i, leg in enumerate(LEGS):
            phase = phases[leg]
            color = LEG_COLORS[leg]
            
            # First part of swing
            swing_end = phase + swing_duration
            if swing_end <= 1.0:
                ax.barh(i, swing_duration, left=phase, height=0.6, 
                       color=color, edgecolor='black', linewidth=1.5)
            else:
                # Swing wraps around
                ax.barh(i, 1.0 - phase, left=phase, height=0.6,
                       color=color, edgecolor='black', linewidth=1.5)
                ax.barh(i, swing_end - 1.0, left=0, height=0.6,
                       color=color, edgecolor='black', linewidth=1.5)
        
        ax.set_xlim(0, 1)
        ax.set_ylim(-0.5, len(LEGS) - 0.5)
        ax.set_yticks(y_positions)
        ax.set_yticklabels(LEGS, fontsize=11, fontweight='bold')
        ax.set_xlabel('Phase (0-1)', fontsize=10)
        ax.set_title(f'{gait_name}\n(Duty Factor: {duty_factor:.0%})', 
                    fontsize=12, fontweight='bold')
        ax.set_xticks([0, 0.25, 0.5, 0.75, 1.0])
        ax.grid(axis='x', linestyle='--', alpha=0.5)
    
    # Add legend
    legend_patches = [mpatches.Patch(color=LEG_COLORS[leg], label=f'{leg} Swing') for leg in LEGS]
    legend_patches.append(mpatches.Patch(color='white', edgecolor='black', label='Stance'))
    fig.legend(handles=legend_patches, loc='lower center', ncol=5, fontsize=10,
               frameon=True, fancybox=True, shadow=True,
               bbox_to_anchor=(0.5, 0.02))
    
    plt.tight_layout(rect=[0, 0.08, 1, 0.95])
    return fig


def plot_circular_phase_diagram():
    """Create circular phase diagrams showing relative leg phases"""
    fig, axes = plt.subplots(2, 3, figsize=(14, 12), subplot_kw={'projection': 'polar'})
    fig.suptitle('Quadruped Gait Circular Phase Diagrams\n(Angular position shows phase offset)', 
                 fontsize=14, fontweight='bold', y=0.98)
    
    axes_flat = axes.flatten()
    
    for idx, (gait_name, gait_data) in enumerate(GAITS.items()):
        ax = axes_flat[idx]
        phases = gait_data['phases']
        
        # Plot each leg as a point on the circle
        for leg in LEGS:
            phase = phases[leg]
            angle = 2 * np.pi * phase  # Convert phase to angle
            
            # Plot point
            ax.scatter(angle, 1.0, s=300, c=LEG_COLORS[leg], 
                      edgecolors='black', linewidths=2, zorder=5)
            
            # Add label
            label_radius = 1.25
            ax.annotate(leg, xy=(angle, label_radius), 
                       fontsize=11, fontweight='bold',
                       ha='center', va='center',
                       color=LEG_COLORS[leg])
            
            # Draw line from center
            ax.plot([angle, angle], [0, 1], color=LEG_COLORS[leg], 
                   linewidth=2, alpha=0.6)
        
        # Formatting
        ax.set_ylim(0, 1.5)
        ax.set_yticks([])
        ax.set_xticks(np.linspace(0, 2*np.pi, 8, endpoint=False))
        ax.set_xticklabels(['0', '', '0.25', '', '0.5', '', '0.75', ''])
        ax.set_title(f'{gait_name}', fontsize=12, fontweight='bold', pad=20)
        ax.grid(True, alpha=0.3)
    
    # Add legend
    legend_patches = [mpatches.Patch(color=LEG_COLORS[leg], label=leg) for leg in LEGS]
    fig.legend(handles=legend_patches, loc='lower center', ncol=4, fontsize=11,
               frameon=True, fancybox=True, shadow=True,
               bbox_to_anchor=(0.5, 0.02))
    
    plt.tight_layout(rect=[0, 0.06, 1, 0.95])
    return fig


def plot_foot_contact_pattern():
    """Plot foot contact pattern over time for each gait"""
    fig, axes = plt.subplots(2, 3, figsize=(16, 10))
    fig.suptitle('Quadruped Foot Contact Pattern (8-DOF Robot)\n● = Foot on Ground (Stance)  ○ = Foot in Air (Swing)', 
                 fontsize=14, fontweight='bold', y=0.98)
    
    axes_flat = axes.flatten()
    time_points = np.linspace(0, 1, 21)  # Sample points in one cycle
    
    for idx, (gait_name, gait_data) in enumerate(GAITS.items()):
        ax = axes_flat[idx]
        phases = gait_data['phases']
        duty_factor = gait_data['duty_factor']
        swing_duration = 1 - duty_factor
        
        for leg_idx, leg in enumerate(LEGS):
            phase_offset = phases[leg]
            color = LEG_COLORS[leg]
            
            for t in time_points:
                # Determine if leg is in swing or stance
                # Stance: from 0 to duty_factor relative to leg's phase
                relative_phase = (t - phase_offset) % 1.0
                
                if relative_phase < swing_duration:
                    # Swing phase - foot in air
                    ax.scatter(t, leg_idx, s=100, facecolors='white', 
                              edgecolors=color, linewidths=2, zorder=3)
                else:
                    # Stance phase - foot on ground
                    ax.scatter(t, leg_idx, s=100, facecolors=color, 
                              edgecolors='black', linewidths=1, zorder=3)
        
        # Formatting
        ax.set_xlim(-0.05, 1.05)
        ax.set_ylim(-0.5, len(LEGS) - 0.5)
        ax.set_yticks(range(len(LEGS)))
        ax.set_yticklabels(LEGS, fontsize=11, fontweight='bold')
        ax.set_xlabel('Phase', fontsize=10)
        ax.set_title(f'{gait_name}\n{gait_data["description"]}', fontsize=12, fontweight='bold')
        ax.set_xticks([0, 0.25, 0.5, 0.75, 1.0])
        ax.grid(True, linestyle='--', alpha=0.3)
        
        # Add horizontal lines
        for i in range(len(LEGS)):
            ax.axhline(y=i, color='lightgray', linestyle='-', linewidth=0.5, zorder=1)
    
    # Add legend
    legend_elements = [
        plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='gray', 
                   markersize=12, label='Stance (Ground)', markeredgecolor='black'),
        plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='white', 
                   markersize=12, label='Swing (Air)', markeredgecolor='gray', markeredgewidth=2)
    ]
    fig.legend(handles=legend_elements, loc='lower center', ncol=2, fontsize=11,
               frameon=True, fancybox=True, shadow=True,
               bbox_to_anchor=(0.5, 0.02))
    
    plt.tight_layout(rect=[0, 0.06, 1, 0.95])
    return fig


def print_gait_summary():
    """Print summary table of all gaits"""
    print("\n" + "="*80)
    print("QUADRUPED GAIT PHASE SUMMARY (8-DOF Robot - No Hip Roll)")
    print("="*80)
    print(f"{'Gait':<10} {'LF':>8} {'RF':>8} {'LH':>8} {'RH':>8} {'Duty':>8} {'Description'}")
    print("-"*80)
    
    for gait_name, gait_data in GAITS.items():
        phases = gait_data['phases']
        duty = gait_data['duty_factor']
        desc = gait_data['description']
        print(f"{gait_name:<10} {phases['LF']:>8.2f} {phases['RF']:>8.2f} "
              f"{phases['LH']:>8.2f} {phases['RH']:>8.2f} {duty:>7.0%}  {desc}")
    
    print("="*80)
    print("\nLeg Convention:")
    print("  LF = Left Front,  RF = Right Front")
    print("  LH = Left Hind,   RH = Right Hind")
    print("\nPhase: 0 to 1 represents one complete gait cycle")
    print("Duty Factor: Fraction of cycle that foot is on ground (stance phase)")
    print("="*80 + "\n")


if __name__ == "__main__":
    # Print summary
    print_gait_summary()
    
    # Create all plots
    fig1 = plot_all_gaits()
    fig2 = plot_gait_timing_bars()
    fig3 = plot_circular_phase_diagram()
    fig4 = plot_foot_contact_pattern()
    
    # Show all figures
    plt.show()
    
    # Optionally save figures
    # fig1.savefig('gait_phase_diagrams.png', dpi=150, bbox_inches='tight')
    # fig2.savefig('gait_timing_bars.png', dpi=150, bbox_inches='tight')
    # fig3.savefig('gait_circular_phases.png', dpi=150, bbox_inches='tight')
    # fig4.savefig('gait_foot_contact.png', dpi=150, bbox_inches='tight')
