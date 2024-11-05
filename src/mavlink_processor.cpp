#include "mavlink_processor.h"
#include <mavlink.h>
#include "motor_driver.h"
#include <HardwareSerial.h>
#include <math.h>

// Constants
#define TARGET_SYSTEM 1
#define TARGET_COMPONENT 154

// Variables for gimbal attitude
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // Neutral attitude

void mavlink_processor_init() {
    Serial.begin(115200);
}

void process_mavlink_message() {
    mavlink_message_t msg;
    mavlink_status_t status;

    while (Serial.available() > 0) {
        uint8_t byte = Serial.read();

        // Parse incoming MAVLink messages
        if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) {
            // Handle specific MAVLink messages
            if (msg.msgid == MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE) {
                mavlink_gimbal_device_set_attitude_t gimbal_attitude;
                mavlink_msg_gimbal_device_set_attitude_decode(&msg, &gimbal_attitude);

                // Extract quaternion for attitude
                q[0] = gimbal_attitude.q[0];
                q[1] = gimbal_attitude.q[1];
                q[2] = gimbal_attitude.q[2];
                q[3] = gimbal_attitude.q[3];

                // Convert quaternion to Euler angles (pitch and yaw) for motor control
                float pitch, roll, yaw;
                // Quaternion to Euler conversion
                float ysqr = q[2] * q[2];

                // Roll (X axis)
                float t0 = +2.0f * (q[0] * q[1] + q[2] * q[3]);
                float t1 = +1.0f - 2.0f * (q[1] * q[1] + ysqr);
                roll = atan2f(t0, t1);

                // Pitch (Y axis)
                float t2 = +2.0f * (q[0] * q[2] - q[3] * q[1]);
                t2 = t2 > 1.0f ? 1.0f : t2;
                t2 = t2 < -1.0f ? -1.0f : t2;
                pitch = asinf(t2);

                // Yaw (Z axis)
                float t3 = +2.0f * (q[0] * q[3] + q[1] * q[2]);
                float t4 = +1.0f - 2.0f * (ysqr + q[3] * q[3]);
                yaw = atan2f(t3, t4);

                // Convert angles from radians to steps (assuming 29090 steps per revolution)
                const float steps_per_rev_x = 29090.0f;
                const float rad_to_steps_x = steps_per_rev_x / (2 * M_PI);

                const float steps_per_rev_y = 9800.0f;
                const float rad_to_steps_y = steps_per_rev_y / (2 * M_PI);

                int desired_position_x = (int)(pitch * rad_to_steps_x);
                int desired_position_y = (int)(yaw * rad_to_steps_y);

                // Calculate steps relative to current position
                int steps_to_move_x = desired_position_x - current_position_x;
                int steps_to_move_y = desired_position_y - current_position_y;

                // Move motors
                motor_set_position_x(steps_to_move_x);
                motor_set_position_y(steps_to_move_y);

                send_gimbal_status();
            }
        }
    }
}

void send_gimbal_status() {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack the GIMBAL_DEVICE_ATTITUDE_STATUS message
    mavlink_msg_gimbal_device_attitude_status_pack(
        TARGET_SYSTEM,                 // System ID
        TARGET_COMPONENT,              // Component ID
        &msg,                          // Message object
        TARGET_SYSTEM,                 // Target system ID
        TARGET_COMPONENT,              // Target component ID
        millis(),                      // Time since boot in milliseconds
        0,                             // Flags
        q,                             // Quaternion
        0.0f,                          // Angular velocity x
        0.0f,                          // Angular velocity y
        0.0f,                          // Angular velocity z
        0,                             // Failure flags
        0.0f,                          // Delta yaw
        0.0f,                          // Delta yaw velocity
        TARGET_COMPONENT               // Gimbal device ID
    );

    // Serialize the message into a buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    // Send the message via UART
    Serial.write(buf, len);
}
