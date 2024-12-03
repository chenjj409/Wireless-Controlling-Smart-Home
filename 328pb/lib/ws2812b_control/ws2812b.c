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

void ws2812b_initialize()
{
    LED_DDR |= (1 << LED_PIN);
}

void ws2812b_demo_flow(int num_led, int sep_ms)
{
    for (int i = 0; i < num_led; ++i)
    {
        for (int j = 0; j < num_led; ++j)
        {
            if (i == j)
            {
                ws2812b_send(255, 255, 255);
            }
            else
            {
                ws2812b_send(0, 0, 0);
            }
        }
        _delay_ms(sep_ms);
    }
}

void ws2812b_demo_color(int num_led, int sep_ms)
{
    int R = 240;
    int G = 0;
    int B = 0;
    int increment = 0;
    while (1)
    {
        if (R == 240)
        {
            increment = 1;
        }
        else if (G == 240)
        {
            increment = 2;
        }
        else if (B == 240)
        {
            increment = 0;
        }
        if (increment == 0)
        {
            R += 20;
            B -= 20;
        }
        else if (increment == 1)
        {
            G += 20;
            R -= 20;
        }
        else
        {
            B += 20;
            G -= 20;
        }
        for (int i = 0; i < num_led; ++ i)
        {
            ws2812b_send(R, G, B);
        }
        _delay_ms(sep_ms);
    }
}

void ws2812b_send(uint16_t Red, uint32_t Green, uint8_t Blue)
{
    uint32_t color = (Green << 16) | (Red << 8) | (Blue);   // Per the datasheet, Green 7 -> 0, then Red 7 -> 0, then Blue 7 -> 0. 
    uint32_t mask = 0b100000000000000000000000;     // same as (1 << 23), but faster

    while (mask > 0)
    {
        ws2812b_send_bit((color & mask));
        mask = mask >> 1;
    }
}

void ws2812b_send_bit(uint32_t bit)
{
    if (bit != 0)
    {
        asm volatile (
            "sbi %[port], %[pb]     \n\t"   // set led pb high
            "nop                    \n\t"   // use nop to delay 0.8us
            "nop                    \n\t"
            "nop                    \n\t"
            "nop                    \n\t"
            "nop                    \n\t"
            "nop                    \n\t"
            "nop                    \n\t"
            "nop                    \n\t"
            "nop                    \n\t"
            "nop                    \n\t"
            "nop                    \n\t"
            "cbi %[port], %[pb]     \n\t"
            "nop                    \n\t"
            "nop                    \n\t"
            "nop                    \n\t"
            "nop                    \n\t"
            ::
            [port]      "I" (_SFR_IO_ADDR(LED_PORT)),
            [pb]        "I" (LED_PIN)
        );
    }
    else
    {
        asm volatile (
            "sbi %[port], %[pb]     \n\t"   // set led pb high
            "nop                    \n\t"   // use nop to delay 0.4us
            "nop                    \n\t"
            "nop                    \n\t"
            "nop                    \n\t"
            "nop                    \n\t"
            "cbi %[port], %[pb]     \n\t"
            "nop                    \n\t"
            "nop                    \n\t"
            "nop                    \n\t"
            "nop                    \n\t"
            :: 
            [port]      "I" (_SFR_IO_ADDR(LED_PORT)), 
            [pb]        "I" (LED_PIN)
        );
    }
}