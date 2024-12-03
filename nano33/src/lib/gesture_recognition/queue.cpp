#include "queue.h"
#include "globals.h"
#include "assert.h"

void initQueue(GyroQueue_t* queue) {
    queue -> front = 0;
    queue -> rear = 0;
    queue -> count = 0;
    queue -> queue_start_idx = 0;
    queue -> queue_end_idx = 0;
}


int  isEmpty(GyroQueue_t* queue) {
    return queue -> count == 0;
}

int isFull(GyroQueue_t* queue) {
    return queue -> count == BUFFERSIZE;
}




int enqueue(GyroQueue_t* queue, SensoryData_t* input_data) {
    if (isFull(queue)) {
        return 0;  // Queue is full
    }
    queue -> data[queue -> rear] = *input_data;            // Add element to the rear
    queue -> rear = (queue -> rear + 1) % BUFFERSIZE;  // Update rear (wrap around)
    queue -> count++;                         // Increase element count
    queue -> queue_end_idx = input_data -> idx;
    return 1;
}



// Dequeue an element
int dequeue(GyroQueue_t* queue, SensoryData_t* output_data) {
    if (isEmpty(queue)) {
        return 0;  // Queue is empty
    }
    SensoryData_t output_data_tmp = queue->data[queue -> front];           // Get the front element
    
    output_data -> idx = output_data_tmp.idx;
    output_data -> gyro_Angular_X = output_data_tmp.gyro_Angular_X;
    output_data -> gyro_Angular_Y = output_data_tmp.gyro_Angular_Y;
    output_data -> gyro_Angular_Z = output_data_tmp.gyro_Angular_Z;
    output_data -> gyro_X = output_data_tmp.gyro_X;
    output_data -> gyro_Y = output_data_tmp.gyro_Y;
    output_data -> gyro_Z = output_data_tmp.gyro_Z;

    queue -> front = (queue -> front + 1) % BUFFERSIZE;  // Update front (wrap around)
    queue -> count--;                         // Decrease element count
    if (queue -> count > 0) {
        queue -> queue_start_idx = queue->data[queue->front].idx;
    }
    return 1;
}


int clear_queue(GyroQueue_t* queue) {
    SensoryData_t output;
    while(!isEmpty(queue)) {
        dequeue(queue, &output);
    }
}


int check_queue_correctness(GyroQueue_t* queue) {
    if (queue == NULL) {
        return 0;
    }

    if (queue->data == NULL) {
        return 0;
    }

    int start_idx = queue -> front;
    int end_idx = queue -> rear;
    if (isEmpty(queue)) {
        if (start_idx != end_idx || queue -> count != 0) {
            return 0;
        } else {
            return 1;
        }
    } else {
        if ((start_idx + queue -> count) % BUFFERSIZE != end_idx) {
            return 0;
        }
        int last_element_data_idx = end_idx - 1;
        if (end_idx == 0) {
            last_element_data_idx = BUFFERSIZE - 1;
        }

        if (queue->data[start_idx].idx != queue->queue_start_idx) {
            return 0;
        }

        if (queue -> data[last_element_data_idx].idx != queue -> queue_end_idx) {
            return 0;
        }

        return 1;
    }
}


void print_queue(GyroQueue_t* queue) {
    int idx_iter = queue->front;
    int element_counter = 0;
    while (element_counter < queue -> count) {
        SensoryData_t sensory_data = queue->data[idx_iter];
        printf("-ELEMENT(%d):[%d, %d, %d, %d, %d, %d]-", sensory_data.idx, sensory_data.gyro_Angular_X, sensory_data.gyro_Angular_Y,  sensory_data.gyro_Angular_Z, sensory_data.gyro_X, sensory_data.gyro_Y, sensory_data.gyro_Z);
        idx_iter = (idx_iter + 1) % BUFFERSIZE;
        element_counter += 1;
    }
    return;
}

static void test_data_insertion() {
    GyroQueue_t queue;
    initQueue(&queue);

    int total_insertions = 100000;
    int inserted_elements;
    SensoryData_t sensor_data_input;
    SensoryData_t sensor_data_output;
    sensor_data_input.idx = 1;
    sensor_data_input.gyro_Angular_X = 1;
    while (inserted_elements < total_insertions) {
        if(!enqueue(&queue, &sensor_data_input)) {
            dequeue(&queue, &sensor_data_output);
            assert(sensor_data_input.idx = sensor_data_output.idx + BUFFERSIZE);
        } else {
            inserted_elements += 1;
        }
    }
}


// int main() {
//     test_data_insertion();
//     printf("test passed");
// }