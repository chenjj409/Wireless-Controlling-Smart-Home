#include <global_variable.h>
#include <avr/io.h>
#include <util/delay.h>
#include <time.h>

void motor_initialize(int angle);
int motor_angle(int angle);