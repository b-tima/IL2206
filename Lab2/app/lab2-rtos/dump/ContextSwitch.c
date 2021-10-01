// File: TwoTasks.c

#include <stdio.h>
#include "includes.h"
#include <string.h>
#include "system.h"
#include "altera_avalon_performance_counter.h"

#include <stdint.h>

#define DEBUG 1

/* Definition of Task Stacks */
/* Stack grows from HIGH to LOW memory */
#define TASK_STACKSIZE 2048
OS_STK task1_stk[TASK_STACKSIZE];
OS_STK task2_stk[TASK_STACKSIZE];
OS_STK stat_stk[TASK_STACKSIZE];

/* Definition of Task Priorities */
#define TASK1_PRIORITY 6 // highest priority
#define TASK2_PRIORITY 7
#define TASK_STAT_PRIORITY 12 // lowest priority

OS_EVENT *sem1;
OS_EVENT *sem2;

OS_EVENT *timer;

uint64_t time_run;

void printStackSize(char *name, INT8U prio)
{
    INT8U err;
    OS_STK_DATA stk_data;

    err = OSTaskStkChk(prio, &stk_data);
    if (err == OS_NO_ERR)
    {
        if (DEBUG == 1)
            printf("%s (priority %d) - Used: %d; Free: %d\n",
                   name, prio, stk_data.OSUsed, stk_data.OSFree);
    }
    else
    {
        if (DEBUG == 1)
            printf("Stack Check Error!\n");
    }
}

/* Prints a message and sleeps for given time interval */
void task1(void *pdata)
{
    uint8_t state = 0;
    uint8_t perr;
    while (1)
    {
        char text1[100];
        OSSemPend(sem1, 0, &perr);
        sprintf(text1, "Task 0 - State %d\n", state);
        int i;
        /*for (i = 0; i < strlen(text1); i++)
            putchar(text1[i]);*/
        if(state == 0) state = 1;
        else state = 0;
        PERF_BEGIN(PERFORMANCE_COUNTER_BASE, 1);
        OSSemPost(sem2);
        OSTimeDlyHMSM(0, 0, 0, 4); /* Context Switch to next task
				   * Task will go to the ready state
				   * after the specified delay
				   */
    }
}

/* Prints a message and sleeps for given time interval */
void task2(void *pdata)
{
    uint8_t state = 0;
    uint8_t perr;
    while (1)
    {
        char text2[100];
        OSSemPend(sem2, 0, &perr);
        PERF_END(PERFORMANCE_COUNTER_BASE, 1);
        sprintf(text2, "Task 1 - State %d\n", state);
        int i;
        /*for (i = 0; i < strlen(text2); i++)
            putchar(text2[i]);*/
        if(state == 0) state = 1;
        else state = 0;
        OSSemPost(sem1);
        OSTimeDlyHMSM(0, 0, 0, 4);
    }
}

/* Printing Statistics */
void statisticTask(void *pdata)
{
    OSTimeDlyHMSM(0, 0, 1, 0);
    PERF_END(PERFORMANCE_COUNTER_BASE, 1);
    PERF_STOP_MEASURING(PERFORMANCE_COUNTER_BASE);
    perf_print_formatted_report(PERFORMANCE_COUNTER_BASE, alt_get_cpu_freq(), 1, "Varmkorvboogie");
    return;
    while (1)
    {
        printStackSize("Task1", TASK1_PRIORITY);
        printStackSize("Task2", TASK2_PRIORITY);
        printStackSize("StatisticTask", TASK_STAT_PRIORITY);
    }
}

/* The main function creates two task and starts multi-tasking */
int main(void)
{
    printf("Lab 3 - Two Tasks\n");

    OSTaskCreateExt(task1,                          // Pointer to task code
                    NULL,                           // Pointer to argument passed to task
                    &task1_stk[TASK_STACKSIZE - 1], // Pointer to top of task stack
                    TASK1_PRIORITY,                 // Desired Task priority
                    TASK1_PRIORITY,                 // Task ID
                    &task1_stk[0],                  // Pointer to bottom of task stack
                    TASK_STACKSIZE,                 // Stacksize
                    NULL,                           // Pointer to user supplied memory (not needed)
                    OS_TASK_OPT_STK_CHK |           // Stack Checking enabled
                        OS_TASK_OPT_STK_CLR         // Stack Cleared
    );

    OSTaskCreateExt(task2,                          // Pointer to task code
                    NULL,                           // Pointer to argument passed to task
                    &task2_stk[TASK_STACKSIZE - 1], // Pointer to top of task stack
                    TASK2_PRIORITY,                 // Desired Task priority
                    TASK2_PRIORITY,                 // Task ID
                    &task2_stk[0],                  // Pointer to bottom of task stack
                    TASK_STACKSIZE,                 // Stacksize
                    NULL,                           // Pointer to user supplied memory (not needed)
                    OS_TASK_OPT_STK_CHK |           // Stack Checking enabled
                        OS_TASK_OPT_STK_CLR         // Stack Cleared
    );

    if (DEBUG == 1)
    {
        OSTaskCreateExt(statisticTask,                 // Pointer to task code
                        NULL,                          // Pointer to argument passed to task
                        &stat_stk[TASK_STACKSIZE - 1], // Pointer to top of task stack
                        TASK_STAT_PRIORITY,            // Desired Task priority
                        TASK_STAT_PRIORITY,            // Task ID
                        &stat_stk[0],                  // Pointer to bottom of task stack
                        TASK_STACKSIZE,                // Stacksize
                        NULL,                          // Pointer to user supplied memory (not needed)
                        OS_TASK_OPT_STK_CHK |          // Stack Checking enabled
                            OS_TASK_OPT_STK_CLR        // Stack Cleared
        );
    }

    //OSInit();

    sem1 = OSSemCreate(1);
    sem2 = OSSemCreate(0);

    //timer = OSTmrCreate();

    PERF_BEGIN(PERFORMANCE_COUNTER_BASE, 1);

    PERF_RESET(PERFORMANCE_COUNTER_BASE);

    //PERF_BEGIN(PERFORMANCE_COUNTER_BASE, 1);
    PERF_START_MEASURING(PERFORMANCE_COUNTER_BASE);

    OSStart();
    return 0;
}
