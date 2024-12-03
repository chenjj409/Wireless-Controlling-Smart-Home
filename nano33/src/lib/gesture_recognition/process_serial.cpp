#include <stdio.h>
#include <stdint.h>
#include "semphore.h"
#include "matrix.h"
#include "queue.h"
#include "globals.h"
#include <Arduino.h>
#include "math.h"

ISR_semaphore_t QUEUESemaphore;

float gyro_buffer_X[100];
float gyro_buffer_Y[100];
float gyro_buffer_Z[100];
float gyro_buffer_AnglularX[100];
float gyro_buffer_AngularY[100];
float gyro_buffer_AngularZ[100];



GyroQueue_t queue_content;
GyroQueue_t* GyroQueue = &queue_content;

static int STABLETIMEGAP = 0;
static int SHARPTURNCOUNTER = 0;
static int TRACKING = 0;
SensoryData_t PriorSensoryData;


void initialize() {
    initQueue(GyroQueue);
    semaphore_init(&QUEUESemaphore, 1);
}



// Use this function in the interrupt handler
void add_to_waitingList(float gyro_x, float gyro_y, float gyro_z, float acc_x, float acc_y, float acc_z, int gyro_idx) {
    int saved_gyro_x = (int)gyro_x * 100.0;
    int saved_gyro_y = (int)gyro_y * 100.0;
    int saved_gyro_z = (int)gyro_y * 100.0;
    int saved_acc_x = (int)acc_x * 100.0;
    int saved_acc_y = (int)acc_y * 100.0;
    int saved_acc_z = (int)acc_z * 100.0;

    // Gyroscope Counter
    SensoryData_t sensory_data;
    sensory_data.idx = gyro_idx;
    sensory_data.gyro_Angular_X = saved_gyro_x;
    sensory_data.gyro_Angular_Y = saved_gyro_y;
    sensory_data.gyro_Angular_Z = saved_gyro_z;
    sensory_data.gyro_X = saved_acc_x;
    sensory_data.gyro_Y = saved_acc_y;
    sensory_data.gyro_Z = saved_acc_z;


    printf("inserting sensory data %d", sensory_data.idx);
    int queue_end_idx = GyroQueue -> queue_end_idx;
    if (isFull(GyroQueue)) {
        dequeue(GyroQueue, &PriorSensoryData);
    }
    if (queue_end_idx + 1 != gyro_idx) {
        clear_queue(GyroQueue);
    } 
    enqueue(GyroQueue, &sensory_data);
    // semaphore_release(&QUEUESemaphore);
}
 
static int compute_angular_Change(SensoryData_t* prior_data, SensoryData_t* current_data) {
    int d_angular_x = current_data -> gyro_Angular_X - prior_data -> gyro_Angular_X;
    int d_angular_y = current_data -> gyro_Angular_Y - prior_data -> gyro_Angular_Y;
    int d_angular_z = current_data -> gyro_Angular_Z - prior_data -> gyro_Angular_Z;
    int angular_shift = sqrt(d_angular_x * d_angular_x + d_angular_y * d_angular_y + d_angular_z * d_angular_z);
    return angular_shift;  
}

static void copy_sensory_data(SensoryData_t* source_data, SensoryData_t* dest_data) {
    dest_data -> idx = source_data -> idx;
    dest_data -> gyro_Angular_X = source_data -> gyro_Angular_X;
    dest_data -> gyro_Angular_Y = source_data -> gyro_Angular_Y;
    dest_data -> gyro_Angular_Z = source_data ->gyro_Angular_Z;
    dest_data -> gyro_X = source_data -> gyro_X;
    dest_data -> gyro_Y = source_data -> gyro_Y;
    dest_data -> gyro_Z = source_data -> gyro_Z;
}



static int execute_calculations(SensoryData_t* current_data, SensoryData_t* prior_data) {
    if (!dequeue(GyroQueue, current_data)) {
        // the queue is currently empty
        return -1;
    } else {
        //the queue is not empty
        int result = -1;
        int prior_idx = prior_data -> idx;
        int current_idx = current_data -> idx;
        if (current_idx != prior_idx + 1) {
            // discontinuous cnt caused by package loss, replace the PriorSensory data with new data
            copy_sensory_data(current_data, prior_data);
            if (TRACKING) {
                STABLETIMEGAP += current_idx - prior_idx;
            }
            char output_str[50];
            sprintf(output_str, "package lost %d - %d\n", current_idx, prior_idx);
            Serial.println(output_str);
        } else {
            // the data is purely continuous
            int angular_shift = compute_angular_Change(prior_data, current_data);
            if (angular_shift > ANGULAR_THREHSOLD) {
                if (TRACKING) {
                    STABLETIMEGAP = 0;
                    SHARPTURNCOUNTER += 1;
                } else {
                    TRACKING = 1;
                    STABLETIMEGAP = 0;
                    SHARPTURNCOUNTER = 0;
                }
                char output_str[50];
                sprintf(output_str, "oberve angular shift %d\n", angular_shift);
                Serial.println(output_str);
            } else {
                if (TRACKING) {
                    STABLETIMEGAP += 1;
                }
                // char output_str[50];
                // sprintf(output_str, "stable state, Tracking %d\n", TRACKING);
                // Serial.println(output_str);
                copy_sensory_data(current_data, prior_data);
            }
        }
        if ((STABLETIMEGAP > GAP_TILL_EXIT) && (TRACKING)) {
            result = SHARPTURNCOUNTER;
            TRACKING = 0;
            //reset the time gap to be zero to allow restarting
            STABLETIMEGAP = 0;
            // char output_str[50];
            // sprintf(output_str, "Ending tracking mode with turn counter %d\n", result);
            // Serial.println(output_str);
            return result;
        } else {
            return 0;
        }
    }
}


int MainTask() {
    SensoryData_t current_data;
    // semaphore_acquire(&QUEUESemaphore);
    // Write function for processing the current data
    int result = execute_calculations(&current_data, &PriorSensoryData);
    if (result > 0) {
        // printf("number of sharp turn counter: %d\n", result);
        // char output_str[50];
        // sprintf(output_str, "Move ended: %d ", result);
        // Serial.println(output_str);
        return result;
    } else {
        // printf("no movement detected\n");
        // char output_str[50];
        // sprintf(output_str, "No moves detected:");
        // Serial.println(output_str);
        return -1;
    }
    
    // open queue to allow insertions
    delay(0.001);
}