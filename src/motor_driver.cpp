#include "motor_driver.h"

// Define global variables for motor positions
int current_position_x = 0;
int current_position_y = 0;

// Pin definitions
#define STEP_PIN_X PB0
#define DIR_PIN_X PB1
#define STEP_PIN_Y PB10
#define DIR_PIN_Y PB11

#define MICROSWITCH_X PA0
#define MICROSWITCH_Y PA1

#define STEPPER_DELAY_Y 200 // Microseconds
#define STEPPER_DELAY_X 100 // Microseconds

void motor_driver_init()
{
    pinMode(STEP_PIN_X, OUTPUT);
    pinMode(DIR_PIN_X, OUTPUT);
    pinMode(STEP_PIN_Y, OUTPUT);
    pinMode(DIR_PIN_Y, OUTPUT);

    pinMode(MICROSWITCH_X, INPUT_PULLUP);
    pinMode(MICROSWITCH_Y, INPUT_PULLUP);

    Serial.begin(115200);
}

bool microswitch_x_pressed()
{
    return digitalRead(MICROSWITCH_X) == LOW;
}

bool microswitch_y_pressed()
{
    return digitalRead(MICROSWITCH_Y) == LOW;
}

void step_motor_x(uint8_t stepPin)
{
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(STEPPER_DELAY_X);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(STEPPER_DELAY_X);
}

void step_motor_y(uint8_t stepPin)
{
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(STEPPER_DELAY_Y);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(STEPPER_DELAY_Y);
}

void motor_set_position_x(int steps)
{
    digitalWrite(DIR_PIN_X, steps > 0 ? HIGH : LOW);
    for (int i = 0; i < abs(steps); i++)
    {
        step_motor_x(STEP_PIN_X);
        current_position_x += (steps > 0) ? 1 : -1;
    }
}

void motor_set_position_y(int steps)
{
    digitalWrite(DIR_PIN_Y, steps > 0 ? HIGH : LOW);
    for (int i = 0; i < abs(steps); i++)
    {
        step_motor_y(STEP_PIN_Y);
        current_position_y += (steps > 0) ? 1 : -1;
    }
}

void homing_sequence()
{
    // Move X axis towards microswitch until it is pressed
    digitalWrite(DIR_PIN_X, HIGH); // Assuming HIGH moves towards microswitch
    int steps = 1000;
    for (int i = 0; i < 2000; i++)
    {
        step_motor_x(STEP_PIN_X);
    }
    
    while (!microswitch_x_pressed())
    {
        step_motor_x(STEP_PIN_X);
        steps++;
    }
    current_position_x = 0; // Zero position established for X
    Serial.print("Steps moved (: ");
    Serial.println(steps);

    // Search for Y home position within +15 to -15 degrees
    // const int steps_per_degree = 400 / 360; // Assuming 200 steps per revolution
    const int max_steps = 2000;
    int steps_moved = 0;

    // Move Y axis towards microswitch until it is pressed
    // digitalWrite(DIR_PIN_Y, LOW); // Assuming LOW moves towards microswitch
    // while (!microswitch_y_pressed())
    // {
    //     step_motor(STEP_PIN_Y);
    // }
    // Move in positive direction first, limited to +15 degrees
    digitalWrite(DIR_PIN_Y, HIGH);
    while (steps_moved < max_steps && !microswitch_y_pressed())
    {
        step_motor_y(STEP_PIN_Y);
        steps_moved++;
    }

    // If microswitch is pressed, set current position
    if (microswitch_y_pressed())
    {
        current_position_y = steps_moved;
        return;
    }

    // Move in negative direction, limited to -15 degrees
    digitalWrite(DIR_PIN_Y, LOW);
    while (steps_moved > -max_steps && !microswitch_y_pressed())
    {
        step_motor_y(STEP_PIN_Y);
        steps_moved--;
    }

    // If microswitch is pressed, set current position
    if (microswitch_y_pressed())
    {
        current_position_y = steps_moved;
        return;
    }

    // If not found, move back to center position
    digitalWrite(DIR_PIN_Y, HIGH);
    for (int i = 0; i < abs(steps_moved); i++)
    {
        step_motor_y(STEP_PIN_Y);
    }
    current_position_y = 0; // Set position back to center
}

void check_y()
{
    const int steps_per_degree = 400 / 360; // Assuming 200 steps per revolution
    const int max_steps = 30 * steps_per_degree;

    digitalWrite(DIR_PIN_X, LOW);
    for (int i = 0; i < max_steps; i++)
    {
        step_motor_x(STEP_PIN_X);
    }
    delayMicroseconds(1000);

    digitalWrite(DIR_PIN_X, HIGH);
    for (int i = 0; i < max_steps; i++)
    {
        step_motor_x(STEP_PIN_X);
    }
    delayMicroseconds(1000);


}
