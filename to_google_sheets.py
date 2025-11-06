#!/usr/bin/env python3
"""
Convert BNO055 dump.txt to Google Sheets compatible CSV
"""

def convert_to_google_sheets(input_file='dump.txt', output_file='bno055_data.csv'):
    """Add header and clean CSV data for Google Sheets import"""
    
    # CSV header
    header = "timestamp,calib_sys,calib_gyro,calib_accel,calib_mag,euler_h,euler_r,euler_p,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z,status\n"
    
    with open(input_file, 'r') as f_in:
        lines = f_in.readlines()
    
    # Filter valid data lines (starting with digit, exactly 18 fields)
    valid_lines = []
    for line in lines:
        line = line.strip()
        if line and line[0].isdigit():
            fields = line.split(',')
            if len(fields) == 18:  # timestamp + 16 values + status
                valid_lines.append(line + '\n')
    
    # Write clean CSV
    with open(output_file, 'w') as f_out:
        f_out.write(header)
        f_out.writelines(valid_lines)
    
    print(f"Google Sheets CSV created!")
    print(f"  Input:  {input_file} ({len(lines)} lines)")
    print(f"  Output: {output_file} ({len(valid_lines)} data lines)")
    print(f"\nUpload '{output_file}' to Google Sheets:")
    print("  1. Open Google Sheets")
    print("  2. File > Import > Upload")
    print("  3. Select 'bno055_data.csv'")
    print("  4. Import location: 'Replace spreadsheet'")
    print("  5. Separator type: 'Comma'")

if __name__ == '__main__':
    convert_to_google_sheets()
