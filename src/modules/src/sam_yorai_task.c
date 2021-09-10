//
// Created by chris on 9/10/21.
//
#include "config.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "static_mem.h"
#include "task.h"
#include <stdbool.h>
#include "sam_yorai_task.h"

struct sam_yorai_task {
    void (*function)(void*);
    void * arg;
};

static xQueueHandle inputQueue;

STATIC_MEM_QUEUE_ALLOC(inputQueue, 1, sizeof(struct sam_yorai_task));

static void samYoraiTask(void*);
STATIC_MEM_TASK_ALLOC(samYoraiTask, SAM_YORAI_TASK_STACKSIZE);

static bool isInit = false;

void samYoraiTaskInit(){
    inputQueue = STATIC_MEM_QUEUE_CREATE(inputQueue);
    STATIC_MEM_TASK_CREATE(samYoraiTask, samYoraiTask, SAM_YORAI_TASK_NAME, NULL, SAM_YORAI_TASK_PRI);
    isInit = true;
}


bool samYoraiTaskTest(){
    return isInit;
}


static void samYoraiTask(void* params){
    DEBUG_PRINT("running forward simulation of quad model");
    while (true){

        if (pdTrue == xQueueRecieve(inputQueue, &input, portMAX_DELAY)){
            //model update here


        }


    }
}

void SamYoraiTaskEnqueueInput(int value){
    xQueueOverwrite(inputQueue, &value);
}





