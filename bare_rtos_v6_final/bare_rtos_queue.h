/*
 * bare_rtos_queue.h
 *
 *  Created on: 26-Apr-2024
 *      Author: basat
 */

#ifndef BARE_RTOS_QUEUE_H_
#define BARE_RTOS_QUEUE_H_

struct queue{
    int element;
    struct queue *next_element;
};

struct queue* bare_rtos_queue_create(int max_size);
void bare_rtos_queue_insert(int value, struct queue *root);
int bare_rtos_queue_get(int index,struct queue *root);
void bare_rtos_queue_insert_indx(int value, int index, struct queue *root);
void bare_rtos_queue_delete(void); // to delete entire queue




#endif /* BARE_RTOS_QUEUE_H_ */
