#include <stdint.h>
#include <stdio.h>
#include <time.h>


typedef struct {
    int height;
    int width;
    int* data;
} Matrix_t;


void matrix_multiply(Matrix_t* A, Matrix_t* B, Matrix_t* C);

void print_matrix(Matrix_t* matrix);






