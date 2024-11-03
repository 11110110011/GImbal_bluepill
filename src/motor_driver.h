#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

// Declare global variables for motor positions (extern to access in mavlink_processor.cpp)
extern int current_position_x;
extern int current_position_y;

void motor_driver_init();
void motor_set_position_x(int steps);
void motor_set_position_y(int steps);
bool microswitch_x_pressed();
bool microswitch_y_pressed();
void homing_sequence();
void check_y();

#endif
