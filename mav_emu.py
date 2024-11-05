import time
import serial
from pymavlink import mavutil

# Replace 'COM3' with your actual COM port
serial_port = 'COM3'
baud_rate = 115200

# Open the serial port
ser = serial.Serial(serial_port, baud_rate, timeout=1)

# Create a MAVLink connection
mavlink_connection = mavutil.mavlink_connection(ser)

def send_gimbal_command(tilt, roll, pan):
    # Create a MAVLink message for gimbal control
    msg = mavlink_connection.mav.mount_control_encode(
        0,  # target system
        0,  # target component
        tilt,  # tilt angle (in centidegrees)
        roll,  # roll angle (in centidegrees)
        pan,  # pan angle (in centidegrees)
        0  # save position (0: no, 1: yes)
    )
    # Send the message
    mavlink_connection.mav.send(msg)

# Example usage: send gimbal command to tilt 1000 centidegrees, roll 0, pan 0
send_gimbal_command(1000, 0, 0)

# Close the serial port
ser.close()