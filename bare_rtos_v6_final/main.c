/*
 * %%%%%%%%% PROJECT NAME -> DEVOLPING RTOS BASED ENVIRONMENT FOR TIVA TM4C123GH6PM
 * PROJECT MEMBERS -> BASATI SIVAKRISHNA
 *                 -> KORNU RAHULDAS
 */

#include <./inc/tm4c123gh6pm.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <driverlib/gpio.h>
#include <driverlib/pin_map.h>
#include <driverlib/sysctl.h>
#include <driverlib/uart.h>
#include <driverlib/systick.h>
#include <driverlib/interrupt.h>
#include <bare_rtos_queue.h>

/*
 * NOTE: Only round robin scheduling is developed for now

 * check_program variable is used to check the features developed
 * check_program = 1 -> basic task switching (with preemptive scheduling algorithm) //debug
 * check_program = 2 -> Intra task communication (Queue)  //debug
 * check_program = 3 -> power saving delay function(ps_delay)  //serial monitor
 * check_program = 4 -> Co-operative scheduling algorithm //debug
 * check_program = 5 -> Custom time period algorithm      //debug
 */

#define check_program 1

#define STACK_SIZE 200
#define TASKS_NUM 5
#define STACK_ALLIGN int32_t
#define XPSR_OFFSET          1
#define PC_OFFSET            2
#define LR_OFFSET            3
#define R12_OFFSET           4
#define R3_OFFSET            5
#define R2_OFFSET            6
#define R1_OFFSET            7
#define R0_OFFSET            8
#define R11_OFFSET           9
#define R10_OFFSET           10
#define R9_OFFSET            11
#define R8_OFFSET            12
#define R7_OFFSET            13
#define R6_OFFSET            14
#define R5_OFFSET            15
#define R4_OFFSET            16
#define KERNEL_STACK_SIZE 100
#define ICSR 0xE000ED04

typedef int32_t * StackPtr;
typedef void *    pVoid;
typedef void (*P2FUNC)(void);
int32_t     KernelStack[KERNEL_STACK_SIZE];
int32_t    *kernelStackPtr = &KernelStack[KERNEL_STACK_SIZE -1 ];
long long int task_1_var = 0, task_2_var = 0, task_3_var = 0;
int queue_var_3 = 0;
int queue_var_2 = 0;
int queue_var_1 = 0;

typedef struct TCB
{
    uint8_t ID;
    uint8_t Priority;
    uint8_t State;
    P2FUNC  TaskCode;
    /*-----Stack pointers-------*/
    StackPtr Sptr;
    /*-----Memory Pointer-------*/
    StackPtr TopStack;
    StackPtr EndStack;
    /*-----Pointer to Next and Previous TCB----*/
    struct TCB *Next_Task;
    struct TCB *Prev_Task;
    /*-----Pointer to Current Queue---*/
    pVoid CurrQueue;
    int delay_value;
    int c_time;
}TCB;

typedef struct
{
    TCB * Front,*Rear;
    int32_t No_Tasks;
}TCBLinkedList;

TCBLinkedList ready_list;
TCB *         pCurrentTask;
struct queue* head_node;

void bare_rtos_Addtask(P2FUNC TaskCode,uint8_t ID,uint8_t Priority,uint32_t StackSize, uint32_t task_time);

void bare_rtos_add_to_head(TCBLinkedList * Queue,TCB *Elem);
void bare_rtos_stack_init(uint32_t StackSize,StackPtr Stack,TCB *Task,uint8_t Flag);

void user_mode(void)
{
    __asm volatile("LDR R0,=kernelStackPtr");
    __asm volatile("LDR R1,[R0]");
    __asm volatile("MSR MSP ,R1");
    pCurrentTask  = ready_list.Front;
    /*-----Set Stack Pointer to PSP---*/
    __asm volatile("MRS R0 , CONTROL");
    __asm volatile("ORR R0,R0,#2");
    __asm volatile("MSR CONTROL,R0");
}


void launch_os(void)
{
    /*----Disable Global Interrupts----*/
    __asm("CPSID I\n");
    //Interrupt disable using asm
    /*--------Load Address of the Current TCB Into R0-----*/
    __asm volatile ("LDR R0, =pCurrentTask");
    /*--------Helping Variables-----*/
    __asm volatile ("LDR R2 ,[R0]");
    __asm volatile ("LDR SP,[R2,#8]");
    /*-------Restore Registers-----*/
    __asm volatile ("POP {R4-R11}");
    __asm volatile ("POP {R0-R3}");
    __asm volatile ("POP {R12}");
    /*-----Skip LR-----*/
    __asm volatile ("ADD SP,SP,#4");
    /*----Create New Start Location----*/
    __asm volatile ("POP {LR}");
    __asm volatile ("ADD SP,SP,#4");
    /*----Restore Stack Pointer----*/
    __asm("CPSIE I\n");                  //Interrupt disable using asm
    __asm volatile ("BX LR");

}

void task_yield(void)
{
       NVIC_INT_CTRL_R |=  NVIC_INT_CTRL_PEND_SV;
       __asm("CPSID I\n");
}


void ps_delay(int n)
{
    pCurrentTask->delay_value = n;
    NVIC_INT_CTRL_R |=  NVIC_INT_CTRL_PEND_SV;
    __asm("CPSID I\n");

}

void delay_ms( int n)
{
    int i, j ;

    for(i = 0; i < n ; i ++)
    {
        for(j = 0; j < 3180; j++);
    }
}


void UART0_Transmitter(unsigned char data)
{
    while((UART0_FR_R & (1<<5)) != 0); /* wait until Tx buffer not full */
    UART0_DR_R = data;                  /* before giving it another byte */
}

void printstring(char *str)
{
  while(*str)
    {
        UART0_Transmitter(*(str++));
    }
}

void uart_init(void)
{
         SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
         SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
         GPIOPinConfigure(GPIO_PA0_U0RX);
         GPIOPinConfigure(GPIO_PA1_U0TX);
         GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
         UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
         printstring("Initialization done\n\r\r");
}

void task_0(void)
{

    while(1)
    {
        delay_ms(10);
    }
}

void task_1(void)
{
    while(1)
           {
#if(check_program == 1 || check_program == 5)
                task_1_var++;
                delay_ms(10);
//                GPIO_PORTF_DATA_R |= 0X02;
//                delay_ms(100);
//                GPIO_PORTF_DATA_R &= (~0X02);
//                delay_ms(100);
#endif
#if(check_program == 2)
          queue_var_1 = bare_rtos_queue_get(1, head_node);
          delay_ms(2000);
          bare_rtos_queue_insert_indx(123, 2, head_node);
          delay_ms(2000);

#endif

#if(check_program == 3)  // checking ps_delay
                printstring("task_1\n\r");
                delay_ms(200);
#endif
#if(check_program == 4)  // co-operative scheduling algo
                task_1_var++;
                task_yield();
                __asm("CPSIE I\n");

#endif

           }
}

void task_2(void)
{
    while(1)
    {
#if(check_program == 1 || check_program == 5)
//        GPIO_PORTF_DATA_R |= 0X04;
//                     delay_ms(300);
//                     GPIO_PORTF_DATA_R &= (~0X04);
//                     delay_ms(300);
        task_2_var++;
        delay_ms(10);
#endif

#if(check_program == 2)   // queue checking
          queue_var_2 = bare_rtos_queue_get(2, head_node);
          delay_ms(2000);
          bare_rtos_queue_insert_indx(456, 3, head_node);
          delay_ms(2000);
#endif

#if(check_program == 3)  // checking ps_delay
          printstring("***task_2***\n\r");
          ps_delay(1500);
          __asm("CPSIE I\n");
#endif
#if(check_program == 4)  // co-operative scheduling algo
                task_2_var++;
                task_yield();
                __asm("CPSIE I\n");

#endif
     }
}

void task_3(void)
{
    while(1)
    {
#if(check_program == 1 || check_program == 5)
          task_3_var++;
          delay_ms(10);
#endif
#if(check_program == 2)   // queue checking
          queue_var_3 = bare_rtos_queue_get(3, head_node);
          delay_ms(2000);
          bare_rtos_queue_insert_indx(789, 1, head_node);
          delay_ms(2000);
#endif

#if(check_program == 3)    // checking ps_delay
          printstring("task_3\n\r");
          delay_ms(200);
#endif
#if(check_program == 4)  // co-operative scheduling algo
                task_3_var++;
                task_yield();
                __asm("CPSIE I\n");
#endif
     }
}


void osRoundRobinScheduler(void)
{


    pCurrentTask = pCurrentTask->Next_Task;
    if(pCurrentTask->ID == 99)
    pCurrentTask = ready_list.Front;

    if((pCurrentTask->delay_value)>0)
    {
        pCurrentTask->delay_value--;
        pCurrentTask = pCurrentTask->Next_Task;
           if(pCurrentTask->ID == 99)
           pCurrentTask = ready_list.Front;
    }

#if(check_program == 5)
    NVIC_ST_CTRL_R &= ~(NVIC_ST_CTRL_ENABLE);
    if(pCurrentTask->Next_Task->ID != 99)
        NVIC_ST_RELOAD_R = (((pCurrentTask->Next_Task->c_time)*16000)) - 1;
    else
        NVIC_ST_RELOAD_R = (((ready_list.Front->c_time)*16000)) - 1;

    NVIC_ST_CTRL_R |= NVIC_ST_CTRL_CLK_SRC | NVIC_ST_CTRL_ENABLE;
#endif
}


void systic_setup(void)
{
    SysTickPeriodSet(16000-1);
#if(check_program != 4)
    SysTickIntEnable();
    SysTickEnable();
#endif
}

void __attribute__((naked)) PendSV_Handler(void)
{
    /*------Context Switch-----*/
                /*3] Save R4-R11 to Stack*/
                /*4] Save new Sp to Stack Pointer in TCB*/
                /*------Before Pushung to Stack Set MSP to PSP Location----*/
                __asm volatile ("MRS R0,PSP");
                __asm volatile ("MOV SP,R0");
                /*-----Push To Task Stack-------*/
                __asm volatile ("PUSH {R4-R11}");
                __asm volatile ("LDR R0, =pCurrentTask");
                __asm volatile ("LDR R1,[R0]");
                __asm volatile ("STR SP,[R1,#8]");


            /*-----Switch To Kernel Stack----*/
            __asm volatile ("LDR R1,=kernelStackPtr");
            __asm volatile ("LDR SP,[R1]");

            __asm volatile ("PUSH {R0,LR}");
            //-----Recall stack frame of function is destroyed after function call i.e SP points to same location before func execution

            __asm volatile ("BL  osRoundRobinScheduler");
            __asm volatile ("POP  {R0,LR}");
            /*------Context Switch of Previous Task-----*/
           /*1] Save R4-R11 to Stack*/
           /*2] Save new Sp to Stack Pointer in TCB*/
           /*--------Context Restore of Next Task------*/
           __asm volatile ("LDR R0, =pCurrentTask");
           __asm volatile ("LDR R1,[R0]");
           __asm volatile ("LDR SP,[R1,#8]");
           __asm volatile ("POP {R4-R11}");
           __asm volatile ("MRS R0 , MSP");
           __asm volatile ("MSR PSP, R0");
           __asm volatile ("BX LR");
}


__attribute__((naked)) void SysTick_Handler(void)
{
    NVIC_INT_CTRL_R |=  NVIC_INT_CTRL_PEND_SV;
    __asm volatile("BX LR");
}

int main (void)
{
    uart_init();
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, 0X0E);
GPIO_PORTF_DATA_R = 0X00;
#if(check_program == 2)

    head_node = bare_rtos_queue_create(10);
    head_node->element = 10;
    bare_rtos_queue_insert(30, head_node);
    bare_rtos_queue_insert(40, head_node);
    bare_rtos_queue_insert(60, head_node);
    bare_rtos_queue_insert(70, head_node);
    bare_rtos_queue_insert(80, head_node);
//
    int get_val = 0;
    get_val = bare_rtos_queue_get(2, head_node);
    bare_rtos_queue_insert_indx(23, 2, head_node);
#endif
    bare_rtos_Addtask(task_0, 99, 4, 100, 1);
    bare_rtos_Addtask(task_1, 2, 5, 100, 2);
    bare_rtos_Addtask(task_2, 3, 6, 100, 4);
    bare_rtos_Addtask(task_3, 4, 7, 100, 6);

    __asm("CPSID I\n");                  //Interrupt disable using asm
    systic_setup();
    user_mode();
    launch_os();
    while(1)
    {

    }

return 0;
}

void bare_rtos_Addtask(P2FUNC TaskCode,uint8_t ID,uint8_t Priority,uint32_t StackSize, uint32_t task_time)
{
    if(ready_list.No_Tasks < TASKS_NUM )
        {
            TCB *TaskN = (TCB*) malloc(1 * sizeof(TCB));
            StackPtr TaskStack = (StackPtr) malloc(StackSize * sizeof(STACK_ALLIGN));

            if (TaskN != NULL)
            {
                TaskN->ID = ID;
                TaskN->Priority = Priority;
                TaskN->TaskCode = TaskCode;
                TaskN->CurrQueue = &ready_list;
                TaskN->delay_value = 0;
                TaskN->c_time = 1;
                #if(check_program == 5)
                TaskN->c_time = task_time;
                #endif
                bare_rtos_add_to_head(&ready_list, TaskN);
                bare_rtos_stack_init(StackSize, TaskStack, TaskN, 0);
            }
        }
}


void bare_rtos_add_to_head(TCBLinkedList * Queue,TCB *Elem)
{
    if(Elem != NULL)
    {
        /*-------Insert New Element----*/
        if(Queue->No_Tasks == 0)
        {
            Queue->Front = Elem;
            Queue->Rear  = Queue->Front;
            Elem->Prev_Task = NULL;
        }
        else
        {
            Elem->Next_Task = Queue->Front;
            Queue->Front->Prev_Task = Elem;
            Queue->Front = Elem;
        }
        Queue->No_Tasks++;
    }
}

void bare_rtos_stack_init(uint32_t StackSize,StackPtr Stack,TCB *Task,uint8_t Flag)
{
    /*----------Init Stack Pointer to point to block below registers-----*/
    Task->Sptr =(int32_t*) (&Stack[STACK_SIZE - R4_OFFSET]);
    Task->TopStack = Stack + StackSize;
    Task->EndStack = Stack;

    /*--------Set T bit to 1------*/
    Stack[STACK_SIZE - XPSR_OFFSET] = (1<<24);

    /*-------Program Counter initialization---*/
    Stack[STACK_SIZE - PC_OFFSET] =(uint32_t) (Task->TaskCode);

    /*--------Init R0->R12-----*/
    Stack[STACK_SIZE - LR_OFFSET] = 0xAAAAAAAA;
    Stack[STACK_SIZE - R12_OFFSET] = 0xAAAAAAAA;
    Stack[STACK_SIZE - R3_OFFSET] = 0xAAAAAAAA;
    Stack[STACK_SIZE - R2_OFFSET] = 0xAAAAAAAA;
    Stack[STACK_SIZE - R1_OFFSET] = 0xAAAAAAAA;
    Stack[STACK_SIZE - R0_OFFSET] = 0xAAAAAAAA;
    Stack[STACK_SIZE - R11_OFFSET] = 0xAAAAAAAA;
    Stack[STACK_SIZE - R10_OFFSET] = 0xAAAAAAAA;
    Stack[STACK_SIZE - R9_OFFSET] = 0xAAAAAAAA;
    Stack[STACK_SIZE - R8_OFFSET] = 0xAAAAAAAA;
    Stack[STACK_SIZE - R7_OFFSET] = 0xAAAAAAAA;
    Stack[STACK_SIZE - R6_OFFSET] = 0xAAAAAAAA;
    Stack[STACK_SIZE - R5_OFFSET] = 0xAAAAAAAA;
    Stack[STACK_SIZE - R4_OFFSET] = 0xAAAAAAAA;
}
