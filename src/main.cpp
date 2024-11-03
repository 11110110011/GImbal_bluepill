#include "motor_driver.h"
#include "mavlink_processor.h"

void setup() {
    motor_driver_init();        // Initialize motor driver
    mavlink_processor_init();   // Initialize MAVLink processor

    homing_sequence();          // Perform homing to establish zero position
    // check_y();
}

void loop() {
    // Check for MAVLink messages and process them
    process_mavlink_message();

    // Other code if necessary

    delay(10); // Small delay to avoid overwhelming the system
}
