#include <global_variable.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <avr/io.h>


uint8_t SPI_SlaveReceive();

void SPI_Slave_initialize()
{
    SPI_DDR |= (1 << SPI_MISO);
    SPCR0 |= (1 << SPE);
}

uint8_t SPI_SlaveReceive()
{
    while(!(SPSR0 & (1<<SPIF)));
    return SPDR0;
}