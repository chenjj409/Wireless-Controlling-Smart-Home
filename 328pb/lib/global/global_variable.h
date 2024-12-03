#include <avr/io.h>

#ifndef F_CPU
#define F_CPU 16000000
#endif

#define LED_DDR DDRB
#define LED_PORT PORTB
#define LED_PIN PB0

#define MOTOR_DDR DDRB
#define MOTOR_PORT PORTB
#define MOTOR_PIN PB1
// Motor use timer 1

#define SPI_DDR DDRB
#define SPI_PORT PORTB
#define SPI_MISO PB4
#define SPI_MOSI PB3
#define SPI_SS PB2
#define SPI_CLK PB5

#define PUMP_DDR DDRC
#define PUMP_PORT PORTC
#define PUMP_PIN PC0