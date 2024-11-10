#include <Arduino.h>
#include <HardwareTimer.h>
#include "motor_driver.h"

#define STEPPER_DELAY_X 100 // Microseconds
#define STEPPER_DELAY_Y 200 // Microseconds

#define STEP_PIN_X PB0
#define DIR_PIN_X PB1
#define STEP_PIN_Y PB10
#define DIR_PIN_Y PB11
#define MICROSWITCH_X PA0
#define MICROSWITCH_Y PA1

volatile int target_position_x = 0;
volatile int target_position_y = 0;
int current_position_x = 0;
int current_position_y = 0;

HardwareTimer timerX(TIM1);
HardwareTimer timerY(TIM2);

// Function declarations
void step_motor_x();
void step_motor_y();
void home_motors();

void motor_driver_init()
{
    pinMode(STEP_PIN_X, OUTPUT);
    pinMode(DIR_PIN_X, OUTPUT);
    pinMode(STEP_PIN_Y, OUTPUT);
    pinMode(DIR_PIN_Y, OUTPUT);

    pinMode(MICROSWITCH_X, INPUT_PULLUP);
    pinMode(MICROSWITCH_Y, INPUT_PULLUP);

    Serial.begin(115200);

    // Initialize timers
    timerX.setOverflow(STEPPER_DELAY_X, MICROSEC_FORMAT);
    timerX.attachInterrupt(step_motor_x);
    timerX.resume();

    timerY.setOverflow(STEPPER_DELAY_Y, MICROSEC_FORMAT);
    timerY.attachInterrupt(step_motor_y);
    timerY.resume();

    Serial.println("Motor driver initialized");

    // Home the motors
    home_motors();
}

bool microswitch_x_pressed()
{
    return digitalRead(MICROSWITCH_X) == LOW;
}

bool microswitch_y_pressed()
{
    return digitalRead(MICROSWITCH_Y) == LOW;
}

void step_motor_x()
{
    static bool step_state = false;
    if (current_position_x != target_position_x)
    {
        digitalWrite(STEP_PIN_X, step_state);
        step_state = !step_state;
        if (!step_state)
        {
            current_position_x += (digitalRead(DIR_PIN_X) == HIGH) ? 1 : -1;
        }
    }
}

void step_motor_y()
{
    static bool step_state = false;
    if (current_position_y != target_position_y)
    {
        digitalWrite(STEP_PIN_Y, step_state);
        step_state = !step_state;
        if (!step_state)
        {
            current_position_y += (digitalRead(DIR_PIN_Y) == HIGH) ? 1 : -1;
        }
    }
}

void motor_set_position_x(int steps)
{
    digitalWrite(DIR_PIN_X, steps > 0 ? HIGH : LOW);
    target_position_x = current_position_x + steps;
    Serial.print("Set target position X: ");
    Serial.println(target_position_x);
}

void motor_set_position_y(int steps)
{
    digitalWrite(DIR_PIN_Y, steps > 0 ? HIGH : LOW);
    target_position_y = current_position_y + steps;
    Serial.print("Set target position Y: ");
    Serial.println(target_position_y);
}

void home_motors()
{
    Serial.println("Homing motors...");

    // Home X axis
    digitalWrite(DIR_PIN_X, LOW); // Move towards the microswitch
    while (!microswitch_x_pressed())
    {
        digitalWrite(STEP_PIN_X, HIGH);
        delayMicroseconds(STEPPER_DELAY_X);
        digitalWrite(STEP_PIN_X, LOW);
        delayMicroseconds(STEPPER_DELAY_X);
    }
    current_position_x = 0;
    target_position_x = 0;
    Serial.println("X axis homed");

    // Home Y axis
    digitalWrite(DIR_PIN_Y, LOW); // Move towards the microswitch
    while (!microswitch_y_pressed())
    {
        digitalWrite(STEP_PIN_Y, HIGH);
        delayMicroseconds(STEPPER_DELAY_Y);
        digitalWrite(STEP_PIN_Y, LOW);
        delayMicroseconds(STEPPER_DELAY_Y);
    }
    current_position_y = 0;
    target_position_y = 0;
    Serial.println("Y axis homed");
}

// void setup() {
//     motor_driver_init();
//     motor_set_position_x(100); // Example target position
//     motor_set_position_y(100); // Example target position
// }

// void loop() {
//     // Add any additional logic here
// }
