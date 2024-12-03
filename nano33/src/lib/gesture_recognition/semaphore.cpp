#include "semphore.h"
#include "sam.h"

#define OVF 0

void semaphore_init(ISR_semaphore_t* semaphore, int max_resources) {
    semaphore -> available_resources = max_resources;
    semaphore -> pending_tasks = 0;
}

void semaphore_acquire(ISR_semaphore_t* semaphore) {
    while (1) {
        //disable time counter 3 
        TC4->COUNT16.CTRLA.bit.ENABLE = 0;
        //disable the time counter interrupt 
        TC4->COUNT16.INTENCLR.reg &= (~(0x1 << OVF));
        // wait for synchronization to complete
        if (semaphore -> available_resources > 0){
            semaphore -> available_resources -= 1;
            
            //enable the timer interrupts
            //enable timer and time counter interrupt signal
            TC4->COUNT16.INTENCLR.reg |= (0x1 << OVF);
            TC4->COUNT16.CTRLA.bit.ENABLE = 1;
            break;
        }


        //enable timer and time counter interrupt signal
        TC4->COUNT16.INTENCLR.reg |= (0x1 << OVF);
        TC4->COUNT16.CTRLA.bit.ENABLE = 1;
    }
}

void semaphore_release(ISR_semaphore_t* semaphore) {
    //disable time counter 3 
    TC4->COUNT16.CTRLA.bit.ENABLE = 0;
    //disable the time counter interrupt 
    TC4->COUNT16.INTENCLR.reg &= (~(0x1 << OVF));

    semaphore -> available_resources += 1;


    //enable timer and time counter interrupt signal
    TC4->COUNT16.INTENCLR.reg |= (0x1 << OVF);
    TC4->COUNT16.CTRLA.bit.ENABLE = 1;
}


