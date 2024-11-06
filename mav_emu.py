import time
from pymavlink import mavutil

# Constants
SERIAL_PORT = 'COM11'  # Replace with your actual COM port
BAUD_RATE = 115200
TARGET_SYSTEM = 1
TARGET_COMPONENT = 1

def send_gimbal_command(mavlink_connection, tilt, roll, pan):
    # Send MOUNT_CONTROL command
    msg = mavlink_connection.mav.mount_control_encode(
        TARGET_SYSTEM,  # Target system ID
        TARGET_COMPONENT,  # Target component ID
        tilt,  # Tilt angle (in centidegrees)
        roll,  # Roll angle (in centidegrees)
        pan,  # Pan angle (in centidegrees)
        0  # Save position (0: no, 1: yes)
    )
    mavlink_connection.mav.send(msg)
    print("Sent MOUNT_CONTROL command.")
    print(msg)

def receive_gimbal_status(mavlink_connection):
    # Wait to receive MOUNT_STATUS
    while True:
        msg = mavlink_connection.recv_match(type='MOUNT_STATUS', blocking=True)
        if msg:
            print("Received MOUNT_STATUS: ", msg)
            break

if __name__ == "__main__":
    try:
        # Open serial port
        mavlink_connection = mavutil.mavlink_connection(
            SERIAL_PORT, baud=BAUD_RATE, source_system=1, source_component=1, use_native=False
        )

        while True:
            # Send command to set gimbal attitude
            send_gimbal_command(mavlink_connection, 90, 100, 200)  # Example: tilt 1000 centidegrees, roll 0, pan 0
            
            # Wait for a short delay
            time.sleep(1)
            
            # Receive feedback from gimbal
            receive_gimbal_status(mavlink_connection)
            
            # Wait before sending next command
            time.sleep(5)
    except KeyboardInterrupt:
        # Close the serial port when exiting
        mavlink_connection.close()