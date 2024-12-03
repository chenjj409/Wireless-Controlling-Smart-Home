#include "matrix.h"
#include "assert.h"



void matrix_multiply(Matrix_t* A, Matrix_t* B, Matrix_t* C) {
    assert((A -> width == B -> height) && (A -> height == C -> height) && (B -> width == C -> width));
    int M = A -> height;
    int N = A -> width;
    int P = B -> width;
    // Initialize result matrix C to zero
    for (int i = 0; i < M; i++) {
        for (int j = 0; j < P; j++) {
            C -> data[j + i * P] = 0.0;
        }
    }

    // Multiply A and B into C
    for (int i = 0; i < M; i++) {
        for (int k = 0; k < N; k++) {
            double temp = A->data[k + i * N]; // Avoid repeated indexing of A[i][k]
            for (int j = 0; j < P; j++) {
                C ->data[j + i * P] += temp * B->data[j + k * P];
            }
        }
    }
}


void print_matrix(Matrix_t* matrix) {
    int rows = matrix -> height;
    int cols = matrix -> width;
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            printf("%d ", matrix->data[i* cols + j]);
        }
        printf("\n");
    }
}



static int test_main() {
    // Define matrix dimensions
    int M = 3, N = 3, P = 3;
    Matrix_t A;
    Matrix_t B;
    Matrix_t C;

    A.height = 3;
    A.width = 3;
    B.height = 3;
    B.width = 3;
    C.height = 3;
    C.width = 3;

    // Define example matrices A and B
    int A_data[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};

    int B_data[9] = {9, 8, 7, 6, 5, 4, 3, 2, 1};
    int C_data[9]; // Result matrix

    A.data = A_data;
    B.data = B_data;
    C.data = C_data;

    // Perform matrix multiplication
    matrix_multiply(&A, &B, &C);

    // Print results
    printf("Matrix A:\n");
    print_matrix(&A);

    printf("Matrix B:\n");
    print_matrix(&B);

    printf("Result Matrix C (A x B):\n");
    print_matrix(&C);

    return 0;
}
