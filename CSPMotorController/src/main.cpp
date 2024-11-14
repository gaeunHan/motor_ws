#include "CSPMotorController.h"
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <time.h>
#include <iostream>
#include <cstdlib>
#include <cstring>

#define MAX_SAFE_STACK (8 * 1024)

// working in parallel with cyclic_task()
char stopSignal;
void *p_function(void *data){
    // get stop signal
    printf("Enter 'q' to stop motor\n");
    scanf("%c", &stopSignal);
};

int main() {
    struct sched_param param = {};
    struct timespec wakeup_time;
    pthread_t stopThread;
    int ret = 0;

    /* Set priority */    
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);

    printf("Using priority %i.\n", param.sched_priority);
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        perror("sched_setscheduler failed");
    }

    /* Lock memory */
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        fprintf(stderr, "Warning: Failed to lock memory: %s\n",
                strerror(errno));
    }

    /* page prefaulting */
    unsigned char dummy[MAX_SAFE_STACK];
    memset(dummy, 0, MAX_SAFE_STACK);

    printf("Starting RT task with dt=%u ns.\n", PERIOD_NS);
    
    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
    wakeup_time.tv_sec += 1; /* start in future */
    wakeup_time.tv_nsec = 0;

    /* generate a motor controller */
    CSPMotorController motorController;

    /* get trajectory params from the user */
    motorController.initTrajectoryParams();

    /* Create a pthread */
    char p1[] = "thread_1";  //thread
    int thr_id;
    pthread_t pthread;

    thr_id = pthread_create(&pthread, NULL, p_function, (void*)p1);
    if(thr_id < 0)
    {
        perror("pthread0 create error");
        exit(EXIT_FAILURE);
    }

    /* Main Task */
    while (1) 
    {
        ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                &wakeup_time, NULL);
        if (ret) 
        {
            fprintf(stderr, "clock_nanosleep(): %s\n", strerror(ret));
            break;
        }

        motorController.cyclicTask();

        if(stopSignal=='q' || motorController.dataNum > 1000) break;

        wakeup_time.tv_nsec += PERIOD_NS;
        while (wakeup_time.tv_nsec >= NSEC_PER_SEC) {
            wakeup_time.tv_nsec -= NSEC_PER_SEC;
            wakeup_time.tv_sec++;
        }
    }

    /* save data to file */
    motorController.saveData("/home/ghan/motor_ws/CSPMotorController/logging/5th_pos01.txt",
                             "/home/ghan/motor_ws/CSPMotorController/logging/5th_vel01.txt");


    return ret;
}
