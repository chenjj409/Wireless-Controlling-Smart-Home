#include <stdlib.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <avr/io.h>
#include <math.h>
#include <time.h>
#include <uart.h>
#include <ws2812b.h>
#include <spi_comm.h>
#include <motor.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)

int door_status = 0; // 0 -> closed, 1 -> opened
int light_status = 0;
int pump_status = 0;

void initialize()
{
    // UART_init(BAUD_PRESCALER);
    ws2812b_initialize();
    SPI_Slave_initialize();
    motor_initialize(0);
    PUMP_DDR |= (1 << PUMP_PIN);
}

int main()
{
    initialize();
    while (1)
    {
        uint8_t data = SPI_SlaveReceive();
        // char String[25];
        // sprintf(String, "%d\r\n", data);
        // UART_putstring(String);
        if (data == 1)  // toggle door
        {
            door_status ^= 1;
            if (door_status)
            {
                motor_angle(90);
            }
            else
            {
                motor_angle(0);
            }
        }
        else if (data == 2) // toggle light 
        {
            light_status ^= 1;
            if (light_status)
            {
                ws2812b_send(255, 255, 255);
                ws2812b_send(255, 255, 255);
                ws2812b_send(255, 255, 255);
            }
            else
            {
                ws2812b_send(0, 0, 0);
                ws2812b_send(0, 0, 0);
                ws2812b_send(0, 0, 0);
            }
        }
        else if (data == 3) // toggle pump
        {
            pump_status ^= 1;
            if (pump_status)
            {
                PUMP_PORT |= (1 << PUMP_PIN);
            }
            else
            {
                PUMP_PORT &= ~(1 << PUMP_PIN);
            }
        }
        
    }
}