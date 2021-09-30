#include <stdint.h>

#define OS_EVENT int
#define OS_ERR_NONE 0

#define SEM1_VALUE 1

OS_EVENT *sem;

int task1(){
    while(1){
        uint8_t perr;
        OSSemPend(sem, 0, &perr);
        if(perr != OS_ERR_NONE){
            return -1;
        }

        // Critical code

        if(OSSemPost(sem) != OS_ERR_NONE){
            return -1;
        }
    }
}

int main(){
    // OSinit

    sem = OSSemCreate(SEM1_VALUE);

    // osstart
}