#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BNO055 CSV Data Visualization Script
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def plot_bno055_csv(csv_file='dump.txt'):
    """Load BNO055 data from CSV and create graphs"""
    
    # Read CSV (find header row)
    with open(csv_file, 'r') as f:
        lines = f.readlines()
    
    # Find header row
    header_idx = None
    for i, line in enumerate(lines):
        if line.startswith('timestamp,'):
            header_idx = i
            break
    
    if header_idx is None:
        print("WARNING: No CSV header - assuming column names")
        # Read data without header (skip corrupted lines)
        import io
        csv_data = io.StringIO(''.join(lines))
        df = pd.read_csv(csv_data, names=[
            'timestamp', 'calib_sys', 'calib_gyro', 'calib_accel', 'calib_mag',
            'euler_h', 'euler_r', 'euler_p',
            'accel_x', 'accel_y', 'accel_z',
            'gyro_x', 'gyro_y', 'gyro_z',
            'mag_x', 'mag_y', 'mag_z', 'status'
        ], on_bad_lines='skip')  # Skip corrupted lines
    else:
        # Read data from header onwards
        df = pd.read_csv(csv_file, skiprows=header_idx, on_bad_lines='skip')
    
    print(f"OK: Loaded {len(df)} samples")
    print(f"    Time range: {df['timestamp'].min()/1000:.1f}s ~ {df['timestamp'].max()/1000:.1f}s")
    print(f"    Calibration: Sys={df['calib_sys'].max()} Gyro={df['calib_gyro'].max()} Accel={df['calib_accel'].max()} Mag={df['calib_mag'].max()}")
    
    # Convert time axis to seconds
    time = df['timestamp'] / 1000.0
    
    # Create 4x2 subplots
    fig, axes = plt.subplots(4, 2, figsize=(16, 12))
    fig.suptitle('BNO055 Sensor Data (CSV)', fontsize=16, fontweight='bold')
    
    # 1. 加速度計
    ax = axes[0, 0]
    ax.plot(time, df['accel_x'], label='X', linewidth=1.5, alpha=0.8)
    ax.plot(time, df['accel_y'], label='Y', linewidth=1.5, alpha=0.8)
    ax.plot(time, df['accel_z'], label='Z', linewidth=1.5, alpha=0.8)
    ax.axhline(y=9.8, color='r', linestyle='--', alpha=0.3, label='Gravity (9.8 m/s²)')
    ax.axhline(y=-9.8, color='r', linestyle='--', alpha=0.3)
    ax.set_ylabel('Acceleration [m/s²]')
    ax.set_title('Accelerometer')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    
    # 2. Gyroscope
    ax = axes[0, 1]
    ax.plot(time, df['gyro_x'], label='X', linewidth=1.5, alpha=0.8)
    ax.plot(time, df['gyro_y'], label='Y', linewidth=1.5, alpha=0.8)
    ax.plot(time, df['gyro_z'], label='Z', linewidth=1.5, alpha=0.8)
    ax.set_ylabel('Angular velocity [deg/s]')
    ax.set_title('Gyroscope')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    
    # 3. Magnetometer
    ax = axes[1, 0]
    ax.plot(time, df['mag_x'], label='X', linewidth=1.5, alpha=0.8)
    ax.plot(time, df['mag_y'], label='Y', linewidth=1.5, alpha=0.8)
    ax.plot(time, df['mag_z'], label='Z', linewidth=1.5, alpha=0.8)
    ax.set_ylabel('Magnetic field [uT]')
    ax.set_title('Magnetometer')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    
    # 4. Euler angles
    ax = axes[1, 1]
    ax.plot(time, df['euler_h'], label='Heading', linewidth=1.5, alpha=0.8)
    ax.plot(time, df['euler_r'], label='Roll', linewidth=1.5, alpha=0.8)
    ax.plot(time, df['euler_p'], label='Pitch', linewidth=1.5, alpha=0.8)
    ax.set_ylabel('Angle [deg]')
    ax.set_title('Euler Angles (Orientation)')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    
    # 5. Acceleration magnitude
    ax = axes[2, 0]
    accel_mag = np.sqrt(df['accel_x']**2 + df['accel_y']**2 + df['accel_z']**2)
    ax.plot(time, accel_mag, linewidth=1.5, color='purple', alpha=0.8)
    ax.axhline(y=9.8, color='r', linestyle='--', alpha=0.3, label='Expected gravity')
    ax.set_ylabel('Magnitude [m/s^2]')
    ax.set_title('Acceleration Magnitude (sqrt(X^2+Y^2+Z^2))')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    
    # 6. Z-axis acceleration (detail)
    ax = axes[2, 1]
    ax.plot(time, df['accel_z'], linewidth=1.5, color='blue', alpha=0.8)
    ax.axhline(y=9.8, color='r', linestyle='--', alpha=0.3, label='Gravity (9.8 m/s^2)')
    ax.axhline(y=-9.8, color='r', linestyle='--', alpha=0.3)
    ax.set_ylabel('Z-axis [m/s^2]')
    ax.set_title('Z-axis Acceleration (Detail)')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    
    # 7. Calibration status
    ax = axes[3, 0]
    ax.plot(time, df['calib_sys'], label='System', linewidth=2, marker='o', markersize=3)
    ax.plot(time, df['calib_gyro'], label='Gyroscope', linewidth=2, marker='s', markersize=3)
    ax.plot(time, df['calib_accel'], label='Accelerometer', linewidth=2, marker='^', markersize=3)
    ax.plot(time, df['calib_mag'], label='Magnetometer', linewidth=2, marker='d', markersize=3)
    ax.set_ylabel('Calibration status')
    ax.set_xlabel('Time [s]')
    ax.set_title('Calibration Status (0=None, 3=Full)')
    ax.set_ylim(-0.5, 3.5)
    ax.legend(loc='upper left')
    ax.grid(True, alpha=0.3)
    
    # 8. Statistics
    ax = axes[3, 1]
    ax.axis('off')
    
    stats_text = f"""
DATA STATISTICS
==================
Samples: {len(df)}
Time range: {df['timestamp'].min()/1000:.1f}s ~ {df['timestamp'].max()/1000:.1f}s
Sampling: {np.mean(np.diff(df['timestamp'])):.1f}ms

=== CALIBRATION ===
System:    {df['calib_sys'].max()}/3
Gyroscope: {df['calib_gyro'].max()}/3
Accel:     {df['calib_accel'].max()}/3
Mag:       {df['calib_mag'].max()}/3

=== ACCEL [m/s^2] ===
X: {df['accel_x'].mean():6.2f} +/- {df['accel_x'].std():.2f}
Y: {df['accel_y'].mean():6.2f} +/- {df['accel_y'].std():.2f}
Z: {df['accel_z'].mean():6.2f} +/- {df['accel_z'].std():.2f}
|A|: {accel_mag.mean():5.2f} +/- {accel_mag.std():.2f}

=== GYRO [deg/s] ===
X: {df['gyro_x'].mean():6.2f} +/- {df['gyro_x'].std():.2f}
Y: {df['gyro_y'].mean():6.2f} +/- {df['gyro_y'].std():.2f}
Z: {df['gyro_z'].mean():6.2f} +/- {df['gyro_z'].std():.2f}

=== MAG [uT] ===
X: {df['mag_x'].mean():7.2f} +/- {df['mag_x'].std():.2f}
Y: {df['mag_y'].mean():7.2f} +/- {df['mag_y'].std():.2f}
Z: {df['mag_z'].mean():7.2f} +/- {df['mag_z'].std():.2f}
"""
    
    ax.text(0.05, 0.95, stats_text, transform=ax.transAxes,
            fontsize=10, verticalalignment='top', family='monospace',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.3))
    
    plt.tight_layout()
    
    # Save
    output_file = 'bno055_csv_plot.png'
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"OK: Graph saved to {output_file}")
    
    plt.show()

if __name__ == '__main__':
    print("BNO055 CSV Data Visualization")
    print("=" * 50)
    plot_bno055_csv('dump.txt')
