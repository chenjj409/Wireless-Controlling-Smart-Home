#include "motor.h"
#include <avr/io.h>
#include <util/delay.h>
#include <time.h>
#include <global_variable.h>

void motor_initialize(int angle);
int adjust_angle(int angle);

// using timer 1
void motor_initialize(int angle)
{
    MOTOR_DDR |= (1 << MOTOR_PIN);
    TCCR1A |= (1 << COM1A1) | (1 << WGM11);
    TCCR1B |= (1 << WGM13) | (1 << CS11);
    ICR1 = 20000;
    OCR1A = angle;
}

int motor_angle(int angle) // angle in degrees
{
    int output = angle / 90 * 1000 + 500;
    OCR1A = output;
}