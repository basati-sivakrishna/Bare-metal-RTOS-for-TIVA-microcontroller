/*
 * bare_rtos_queue.c
 *
 *  Created on: 26-Apr-2024
 *      Author: basat
 */
#include<bare_rtos_queue.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

struct queue* bare_rtos_queue_create(int max_size)
{
struct queue* queue_head = (struct queue*)malloc(sizeof(struct queue));
queue_head->element = 0;
queue_head->next_element = NULL;
return queue_head;
}

void bare_rtos_queue_insert(int value, struct queue *root)
{
    while(root->next_element != NULL)
        root = root->next_element;
    struct queue* temp = (struct queue*)malloc(sizeof(struct queue));
    temp->element = value;
    temp->next_element = NULL;
    root->next_element = temp;

}

int bare_rtos_queue_get(int index, struct queue *root)
{
    int ret_value = 0;
    while(index >= 0 && root->next_element != NULL)
    {
     ret_value = root->element;
     root = root->next_element ;
        index--;
    }
    return ret_value;

}

void bare_rtos_queue_insert_indx(int value, int index, struct queue *root)
{
    while(index != 0 && root->next_element != NULL)
    {
        root = root->next_element ;
        index--;
    }
    root->element = value;
}




