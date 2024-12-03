#include <stdint.h>
#include <stdio.h>

#ifndef GYRO_H
#define GYRO_H

typedef struct{
    int idx;
    int gyro_Angular_X;
    int gyro_Angular_Y;
    int gyro_Angular_Z;
    int gyro_X;
    int gyro_Y;
    int gyro_Z;
} SensoryData_t;

typedef enum{
    TRIANGLE,
    CIRCLE,
    SQUARE,
} Shape_t;

#endif //QUEUE_H





// float* denoise_gyroscope_data(SensoryData_t* sensors, int length);


// float* FFT_gyroscope_data(SensoryData_t* sensors, int length);


// float* perform_matrix_multiplication(SensoryData_t* sensors, int length);



// float* thresholding_detection(SensoryData_t* sensors, int length);
