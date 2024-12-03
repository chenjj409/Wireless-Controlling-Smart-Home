#include <global_variable.h>
#include <stdlib.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/io.h>

void ws2812b_send_bit(uint32_t bit);
void ws2812b_send(uint16_t Red, uint32_t Green, uint8_t Blue);
void ws2812b_demo_color(int num_led, int sep_ms);
void ws2812b_demo_flow(int num_led, int sep_ms);
void ws2812b_initialize();
