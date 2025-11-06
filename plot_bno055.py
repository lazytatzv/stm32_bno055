#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BNO055 センサーデータ可視化スクリプト
dump.txt からデータを読み込んでグラフ化
"""

import re
import matplotlib.pyplot as plt
import numpy as np

def parse_dump_file(filename):
    """dump.txtファイルを解析してデータを抽出"""
    data = {
        'accel_x': [], 'accel_y': [], 'accel_z': [],
        'gyro_x': [], 'gyro_y': [], 'gyro_z': [],
        'mag_x': [], 'mag_y': [], 'mag_z': [],
        'euler_h': [], 'euler_r': [], 'euler_p': [],
        'calib_sys': [], 'calib_gyro': [], 'calib_accel': [], 'calib_mag': []
    }
    
    with open(filename, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # 各データブロックを抽出
    blocks = re.split(r'=== BNO055 Data', content)
    
    for block in blocks[1:]:  # 最初の空ブロックをスキップ
        # 各パラメータを一時変数に格納
        temp_data = {}
        
        # Calibration
        calib_match = re.search(r'Calib: Sys=(\d+) Gyro=(\d+) Accel=(\d+) Mag=(\d+)', block)
        if calib_match:
            temp_data['calib_sys'] = int(calib_match.group(1))
            temp_data['calib_gyro'] = int(calib_match.group(2))
            temp_data['calib_accel'] = int(calib_match.group(3))
            temp_data['calib_mag'] = int(calib_match.group(4))
        
        # Euler angles
        euler_match = re.search(r'Euler - H:([\d.-]+) R:([\d.-]+) P:([\d.-]+)', block)
        if euler_match:
            temp_data['euler_h'] = float(euler_match.group(1))
            temp_data['euler_r'] = float(euler_match.group(2))
            temp_data['euler_p'] = float(euler_match.group(3))
        
        # Accelerometer
        accel_match = re.search(r'Accel - X:([\d.-]+) Y:([\d.-]+) Z:([\d.-]+)', block)
        if accel_match:
            temp_data['accel_x'] = float(accel_match.group(1))
            temp_data['accel_y'] = float(accel_match.group(2))
            temp_data['accel_z'] = float(accel_match.group(3))
        
        # Gyroscope
        gyro_match = re.search(r'Gyro\s+- X:([\d.-]+) Y:([\d.-]+) Z:([\d.-]+)', block)
        if gyro_match:
            temp_data['gyro_x'] = float(gyro_match.group(1))
            temp_data['gyro_y'] = float(gyro_match.group(2))
            temp_data['gyro_z'] = float(gyro_match.group(3))
        
        # Magnetometer
        mag_match = re.search(r'Mag\s+- X:([\d.-]+) Y:([\d.-]+) Z:([\d.-]+)', block)
        if mag_match:
            temp_data['mag_x'] = float(mag_match.group(1))
            temp_data['mag_y'] = float(mag_match.group(2))
            temp_data['mag_z'] = float(mag_match.group(3))
        
        # すべてのセンサーデータが揃っている場合のみ追加
        if all(key in temp_data for key in ['accel_x', 'gyro_x', 'mag_x', 'euler_h', 'calib_sys']):
            for key in data.keys():
                data[key].append(temp_data[key])
    
    return data

def plot_sensor_data(data):
    """センサーデータをグラフ化"""
    # サンプル数
    n_samples = len(data['accel_x'])
    time = np.arange(n_samples) * 0.1  # 100ms間隔
    
    # 4x2のサブプロット作成
    fig, axes = plt.subplots(4, 2, figsize=(14, 12))
    fig.suptitle('BNO055 Sensor Data Visualization', fontsize=16, fontweight='bold')
    
    # 1. Accelerometer
    ax = axes[0, 0]
    ax.plot(time, data['accel_x'], label='X', linewidth=1.5)
    ax.plot(time, data['accel_y'], label='Y', linewidth=1.5)
    ax.plot(time, data['accel_z'], label='Z', linewidth=1.5)
    ax.axhline(y=9.8, color='r', linestyle='--', alpha=0.5, label='Gravity (9.8 m/s²)')
    ax.set_ylabel('Acceleration (m/s²)')
    ax.set_title('Accelerometer')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    
    # 2. Gyroscope
    ax = axes[0, 1]
    ax.plot(time, data['gyro_x'], label='X', linewidth=1.5)
    ax.plot(time, data['gyro_y'], label='Y', linewidth=1.5)
    ax.plot(time, data['gyro_z'], label='Z', linewidth=1.5)
    ax.axhline(y=0, color='k', linestyle='-', alpha=0.3)
    ax.set_ylabel('Angular Velocity (deg/s)')
    ax.set_title('Gyroscope')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    
    # 3. Magnetometer
    ax = axes[1, 0]
    ax.plot(time, data['mag_x'], label='X', linewidth=1.5)
    ax.plot(time, data['mag_y'], label='Y', linewidth=1.5)
    ax.plot(time, data['mag_z'], label='Z', linewidth=1.5)
    ax.set_ylabel('Magnetic Field (µT)')
    ax.set_title('Magnetometer')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    
    # 4. Euler Angles
    ax = axes[1, 1]
    ax.plot(time, data['euler_h'], label='Heading', linewidth=1.5)
    ax.plot(time, data['euler_r'], label='Roll', linewidth=1.5)
    ax.plot(time, data['euler_p'], label='Pitch', linewidth=1.5)
    ax.set_ylabel('Angle (degrees)')
    ax.set_title('Euler Angles (Orientation)')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    
    # 5. Calibration Status
    ax = axes[2, 0]
    ax.plot(time, data['calib_sys'], label='System', marker='o', linewidth=1.5)
    ax.plot(time, data['calib_gyro'], label='Gyro', marker='s', linewidth=1.5)
    ax.plot(time, data['calib_accel'], label='Accel', marker='^', linewidth=1.5)
    ax.plot(time, data['calib_mag'], label='Mag', marker='d', linewidth=1.5)
    ax.set_ylabel('Calibration Level (0-3)')
    ax.set_xlabel('Time (seconds)')
    ax.set_title('Calibration Status')
    ax.set_ylim(-0.5, 3.5)
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    
    # 6. Accelerometer Z-axis (詳細)
    ax = axes[2, 1]
    ax.plot(time, data['accel_z'], linewidth=2, label='Z-axis')
    ax.axhline(y=9.8, color='r', linestyle='--', alpha=0.7, linewidth=2, label='Ideal Gravity')
    ax.fill_between(time, 9.5, 10.1, alpha=0.2, color='green', label='±0.3 m/s² range')
    ax.set_ylabel('Z Acceleration (m/s²)')
    ax.set_xlabel('Time (seconds)')
    ax.set_title('Z-Axis Detail (Gravity Component)')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    ax.set_ylim(8, 11)
    
    # 7. Accelerometer 統計
    ax = axes[3, 0]
    stats_text = f"""Accelerometer Statistics:
    
X-axis: Mean={np.mean(data['accel_x']):.3f}, Std={np.std(data['accel_x']):.3f} m/s²
Y-axis: Mean={np.mean(data['accel_y']):.3f}, Std={np.std(data['accel_y']):.3f} m/s²
Z-axis: Mean={np.mean(data['accel_z']):.3f}, Std={np.std(data['accel_z']):.3f} m/s²

Gyroscope Statistics:
X-axis: Mean={np.mean(data['gyro_x']):.3f}, Std={np.std(data['gyro_x']):.3f} deg/s
Y-axis: Mean={np.mean(data['gyro_y']):.3f}, Std={np.std(data['gyro_y']):.3f} deg/s
Z-axis: Mean={np.mean(data['gyro_z']):.3f}, Std={np.std(data['gyro_z']):.3f} deg/s

Total Samples: {n_samples}
Duration: {time[-1]:.1f} seconds"""
    
    ax.text(0.05, 0.95, stats_text, transform=ax.transAxes,
            fontsize=10, verticalalignment='top', fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    ax.axis('off')
    ax.set_title('Statistics Summary')
    
    # 8. Magnetometer XY平面 (Compass view)
    ax = axes[3, 1]
    ax.scatter(data['mag_x'], data['mag_y'], c=time, cmap='viridis', s=30, alpha=0.6)
    ax.set_xlabel('Mag X (µT)')
    ax.set_ylabel('Mag Y (µT)')
    ax.set_title('Magnetometer XY Plane (Compass View)')
    ax.grid(True, alpha=0.3)
    ax.axhline(y=0, color='k', linestyle='-', alpha=0.3)
    ax.axvline(x=0, color='k', linestyle='-', alpha=0.3)
    ax.set_aspect('equal', adjustable='box')
    
    plt.tight_layout()
    return fig

def main():
    """メイン処理"""
    print("BNO055 Data Visualization Tool")
    print("=" * 50)
    
    # データ読み込み
    print("Reading dump.txt...")
    try:
        data = parse_dump_file('dump.txt')
        n_samples = len(data['accel_x'])
        print(f"✓ Successfully parsed {n_samples} data samples")
    except FileNotFoundError:
        print("✗ Error: dump.txt not found!")
        return
    except Exception as e:
        print(f"✗ Error parsing file: {e}")
        return
    
    if n_samples == 0:
        print("✗ No data found in dump.txt")
        return
    
    # グラフ作成
    print("Generating plots...")
    fig = plot_sensor_data(data)
    
    # 保存
    output_file = 'bno055_plot.png'
    fig.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"✓ Plot saved as: {output_file}")
    
    # 表示
    print("✓ Displaying plot...")
    plt.show()

if __name__ == '__main__':
    main()
