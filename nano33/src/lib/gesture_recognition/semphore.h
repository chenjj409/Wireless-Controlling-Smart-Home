#include <stdint.h>

typedef struct {
    int available_resources;
    volatile int pending_tasks;
} ISR_semaphore_t;

// Initialize a semaphore
void semaphore_init(ISR_semaphore_t* semaphore, int max_resources);

// wait for the resources if there aren't any resources left, otherwise wait for the resources
void semaphore_acquire(ISR_semaphore_t* semaphore);

// release the resources 
void semaphore_release(ISR_semaphore_t* semaphore);