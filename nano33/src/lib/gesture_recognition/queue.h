#include <stdint.h>
#include <stdio.h>
#include "Gyro.h"
#include "globals.h"


#ifndef QUEUE_H
#define QUEUE_H

typedef struct {
    SensoryData_t data[BUFFERSIZE];  // Array to store data
    int front;             // Index of the front element
    int rear;              // Index of the rear element
    int count;             // Number of elements in the queue
    int queue_start_idx;
    int queue_end_idx;
}GyroQueue_t;

#endif //QUEUE_H



// Initialize the queue for storing gyroscope data
void initQueue(GyroQueue_t* queue);

// Check if the queue is empty or not 
int isEmpty(GyroQueue_t* queue);


// Check if the queue reached its maximal capacity
int isFull(GyroQueue_t* queue);

//Insert a new element into the queue, return False if it's full
int enqueue(GyroQueue_t* queue, SensoryData_t* input_data);

//pop out the first element from the queue, return false if the queue is empty
int dequeue(GyroQueue_t* queue, SensoryData_t* output_data);

// remove all elements from the queue and reinitialize
int clear_queue(GyroQueue_t* queue);




