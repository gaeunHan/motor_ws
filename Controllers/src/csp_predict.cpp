#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h> /* clock_gettime() */
#include <sys/mman.h> /* mlockall() */
#include <sys/stat.h>
#include <sched.h> /* sched_setscheduler() */
#include <pthread.h>
#include <math.h>
#include <stdbool.h> /* boolean */
#include <cmath>
#include <iostream>
#include <fstream> /* file */
#include <vector>
#include <string>
#include <fcntl.h>
#include <iomanip> // std::fixed, std::setprecision
#include <mutex>
#include "ecrt.h"
#include "MotorController.h"
#include "shared_memory.hpp"

using namespace std;

/****************************************************************************/
#define PERIOD_NS   (1000000) /** Task period in ns. */
#define MAX_SAFE_STACK (8 * 1024) /* The maximum stack size which is guranteed safe to access without faulting */
#define NSEC_PER_SEC (1000000000)
#define FREQUENCY (NSEC_PER_SEC / PERIOD_NS)
#define MAXON_EPOS4_5A 0x000000fb, 0x61500000 // Product Number 확인 필요(ESI file)
#define MOTION_INPUT_PERIOD 1.0 // sec
enum Mode{CSP_ZERO_VEL, CSV_PREV_VEL, CSP_PREDICT};
string basePath = "/home/robogram/motor_ws/Controllers/logging/csp_predict/";

/****************************************************************************/
// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

static ec_slave_config_t *slave_config = NULL;
static ec_slave_config_state_t slave_config_state = {};
/****************************************************************************/

// process data
static uint8_t *domain1_pd = NULL; 
static unsigned int counter = 0;

// RxPDO (master -> slave) offsets for PDO entries
static unsigned int offset_control_word;
static unsigned int offset_target_position;
static unsigned int offset_position_offset;
static unsigned int offset_torque_offset;
static unsigned int offset_modes_of_operation;
static unsigned int offset_digital_outputs;
static unsigned int offset_min_position_limit;
static unsigned int offset_max_position_limit;    

// TxPDO (slave -> master) offsets for PDO entries 
static unsigned int offset_status_word;
static unsigned int offset_position_actual_value;
static unsigned int offset_velocity_actual_value;
static unsigned int offset_torque_actual_value;
static unsigned int offset_modes_of_operation_display;
static unsigned int offset_digital_inputs;
static unsigned int offset_epos4_error_code;
    

// MDP Module CSP
const static ec_pdo_entry_reg_t domain1_regs[] = 
{
    // RxPDO
    {0,0, MAXON_EPOS4_5A, 0x6040, 0x00, &offset_control_word},
    {0,0, MAXON_EPOS4_5A, 0x607A, 0x00, &offset_target_position},
    {0,0, MAXON_EPOS4_5A, 0x60B0, 0x00, &offset_position_offset},
    {0,0, MAXON_EPOS4_5A, 0x60B2, 0x00, &offset_torque_offset},
    {0,0, MAXON_EPOS4_5A, 0x6060, 0x00, &offset_modes_of_operation},
    {0,0, MAXON_EPOS4_5A, 0x60FE, 0x01, &offset_digital_outputs},
    {0,0, MAXON_EPOS4_5A, 0x607D, 0x01, &offset_min_position_limit},
    {0,0, MAXON_EPOS4_5A, 0x607D, 0x02, &offset_max_position_limit},

    // TxPDO
    {0,0, MAXON_EPOS4_5A, 0x6041, 0x00, &offset_status_word},
    {0,0, MAXON_EPOS4_5A, 0x6064, 0x00, &offset_position_actual_value},
    {0,0, MAXON_EPOS4_5A, 0x606C, 0x00, &offset_velocity_actual_value},
    {0,0, MAXON_EPOS4_5A, 0x6077, 0x00, &offset_torque_actual_value},
    {0,0, MAXON_EPOS4_5A, 0x6061, 0x00, &offset_modes_of_operation_display},
    {0,0, MAXON_EPOS4_5A, 0x60FD, 0x00, &offset_digital_inputs},
    {0,0, MAXON_EPOS4_5A, 0x603F, 0x00, &offset_epos4_error_code}
};


/**************************** MDP module CSP mapping ****************************/
static ec_pdo_entry_info_t csp_pdo_entries[] = {
    // RxPDO (Master -> Slave)
    {0x6040, 0x00, 16},    // control word
    {0x607A, 0x00, 32},    // target position
    {0x60B0, 0x00, 32},    // position offset
    {0x60B2, 0x00, 16},    // torque offset
    {0x6060, 0x00, 8},     // modes of operation
    {0x60FE, 0x01, 32},    // digital outputs
    {0x607D, 0x01, 32},    // min position limit
    {0x607D, 0x02, 32},    // max position limit

    // TxPDO (Slave -> Master)
    {0x6041, 0x00, 16},    // status word
    {0x6064, 0x00, 32},    // position actual value
    {0x606C, 0x00, 32},    // velocity actual value
    {0x6077, 0x00, 16},    // torque actual value
    {0x6061, 0x00, 8},     // modes of operation display
    {0x60FD, 0x00, 32},     // digital inputs
    {0x603F, 0x00, 16}     // error code
};

static ec_pdo_info_t csp_pdos[] = {
    // RxPDO(Master -> Slave) 1 mapping
    {0x1600, 8,	csp_pdo_entries + 0}, // 8개의 RxPDO entry를 mapping 할 것인데, entry의 시작 위치는 maxon_epos4_pdo_entries[0]이다. 

    // TxPDO(Master <- Slave) 1 mapping
    {0x1a00, 7,	csp_pdo_entries + 8} // 7개의 TxPDO entry를 mapping 할 것인데, entry의 시작 위치는 maxon_epos4_pdo_entries[8]이다. 
};

// slave sync manager
// EC_DIR_OUTPUT: Master -> Slave, EC_DIR_INPUT: Master <- Slave
static ec_sync_info_t maxon_epos4_syncs_csp[] = {
	{ 0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE },
	{ 1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE },
	{ 2, EC_DIR_OUTPUT, 1, csp_pdos + 0, EC_WD_ENABLE },
	{ 3, EC_DIR_INPUT,  1, csp_pdos + 1, EC_WD_DISABLE },
	{ 0xff }
};

/*****************************************************************************/

void check_domain1_state(void)
{
    ec_domain_state_t ds;

    ecrt_domain_state(domain1, &ds);

    if (ds.working_counter != domain1_state.working_counter) {
        printf("Domain1: WC %u.\n", ds.working_counter);
    }
    if (ds.wc_state != domain1_state.wc_state) {
        printf("Domain1: State %u.\n", ds.wc_state);
    }

    domain1_state = ds;
}

/*****************************************************************************/

void check_master_state(void)
{
    ec_master_state_t ms;
    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding) {
        printf("%u slave(s).\n", ms.slaves_responding);
    }
    if (ms.al_states != master_state.al_states) {
        printf("AL states: 0x%02X.\n", ms.al_states);
    }
    if (ms.link_up != master_state.link_up) {
        printf("Link is %s.\n", ms.link_up ? "up" : "down");
    }

    master_state = ms;
}

/*****************************************************************************/

void check_slave_config_states(void)
{
    ec_slave_config_state_t s;

    ecrt_slave_config_state(slave_config, &s);

    if (s.al_state != slave_config_state.al_state) {
        printf("EPOS4: State 0x%02X.\n", s.al_state);
    }
    if (s.online != slave_config_state.online) {
        printf("EPOS4: %s.\n", s.online ? "online" : "offline");
    }
    if (s.operational != slave_config_state.operational) {
        printf("EPOS4: %soperational.\n", s.operational ? "" : "Not ");
    }

    slave_config_state = s;
}
/*****************************************************************************/
// get target position from the shared memory
float target_position;
SharedMemoryData *shared_memory = nullptr;

void getTargetPos(){
    pthread_mutex_lock(&shared_memory->mutex);

    // 데이터 준비 상태 확인
    while (!shared_memory->data_ready)
    {
        pthread_cond_wait(&shared_memory->condition, &shared_memory->mutex);
    }

    // 데이터 읽기
    target_position = shared_memory->data;
    shared_memory->data_ready = false; // 플래그 초기화

    pthread_mutex_unlock(&shared_memory->mutex);

    // debugging
    //std::cout << std::fixed << std::setprecision(1) << "Motor Control: Received Target Position = " << target_position << std::endl;
}
/*****************************************************************************/
// define motor function: logging, cyclic task ..
// motor object
EPOS4Slave motor1(1024.0, 35.0);

// trajectories
float motionTick = 0.0;
float currPosDeg = 0.0;
float positions[2] = {0.0, 0.0};
int getPosNum = 0;

// logging vars.
float tick = 0;
int idx = 0;
float command_time = 0.0;

// cyclic task vars.
uint16_t status_word = 0;
uint16_t prev_status_word = -1;
uint16_t check_status_word;
uint16_t control_word = 0;
uint16_t epos4_error_code = 0;
uint32_t target_velocity = 0;
bool is_init = 0;
bool is_operational = 0;
bool is_logging = 0;
bool is_stop = 0;
bool is_shutdown = 0;
bool is_terminate = 0;
bool is_new_command = 1;
int N = 1;

// 1ms period
void cyclic_task_csp()
{
    // (마스터가) receive process data
    ecrt_master_receive(master); // 데이터 받아(queue에서 데이터그램 dispatch)
    ecrt_domain_process(domain1); // 정상적인 통신 이루어졌는지 확인, 도메인 상태 업뎃

    // check process data state - optional
    check_domain1_state(); // ecrt_domain_state(), 도메인 상태 확인 

    if (counter) 
    {
        counter--;   
    } 
    else // 1sec period
    { 
        counter = FREQUENCY;  
        
        // check for master state
        check_master_state(); // ecrt_master_state()

        // check for slave configuration state(s)
        check_slave_config_states(); // ecrt_slave_config_state()

        // get statusword
        status_word = EC_READ_U16(domain1_pd + offset_status_word);
        
        // write process data   
        check_status_word = status_word & 0x006F; // check 0,1,2,3,5,6th bit only
        switch(check_status_word){
            case 0x0040: // switch on disabled
                EC_WRITE_U16(domain1_pd + offset_control_word, 0x0006);
                printf("switch on disabled -> ready to switch on\n");
                break;

            case 0x0021: // ready to switch on
                EC_WRITE_U16(domain1_pd + offset_control_word, 0x0007);
                printf("ready to switch on -> switched on\n");
                break;

            case 0x0023: // switched on
                EC_WRITE_U16(domain1_pd + offset_control_word, 0x000F);
                printf("switched on -> operation enabled\n");
                break;

            case 0x0027: // operation enabled
                if(!is_init){                    
                    EC_WRITE_U16(domain1_pd + offset_modes_of_operation, 8); // select csp mode
                    EC_WRITE_U32(domain1_pd + offset_min_position_limit, -1000000000); // set min pos limit
                    EC_WRITE_U32(domain1_pd + offset_max_position_limit, 1000000000); // set max pos limit
                    
                    is_init = 1;            
                    is_logging = 1;       
                } 

                // get target position: 1초 주기의 수술로봇 팔 모션
                getTargetPos();
                getPosNum++;
                positions[0] = positions[1];
                positions[1] = target_position;

                // bring one motion input delay
                if(getPosNum >= 2){
                    is_operational = 1;
                    // get current position in deg
                    currPosDeg = (float)EC_READ_S32(domain1_pd + offset_position_actual_value) * 360.0 / motor1.getCntPerRevolution();
                    // update vel[0]
                    motor1.setVel0();
                    // set trajectory: 1초 주기의 수술로봇 팔 모션 생성
                    motor1.setTrajectoryParam(currPosDeg, positions[0], positions[1], motionTick, MOTION_INPUT_PERIOD, CSP_PREDICT);
                }
                
                motionTick++; // 1초에 1씩 증가
                
                break;

            case 0x0008: //fault
                // read error code
                epos4_error_code = EC_READ_U16(domain1_pd + offset_epos4_error_code);
                printf("Fault, error code is: 0x%04X\n", epos4_error_code);

                // get controlword
                control_word = EC_READ_U16(domain1_pd + offset_control_word);

                // fault reset
                control_word |= 0x0080; 
                EC_WRITE_U16(domain1_pd + offset_control_word, control_word);

                // following error handling
                if((epos4_error_code & 0xFFFF) == 0x8611){
                    EC_WRITE_U16(domain1_pd + offset_modes_of_operation, 6); // select homing mode
                    printf("Homing mode is selected\n");

                    EC_WRITE_U16(domain1_pd + offset_control_word, control_word |= 0x0010);

                    // check if homing is done
                    while(!(EC_READ_U16(domain1_pd + offset_status_word) & 0xF000)) {
                        printf("homing..\n");
                    }
                    printf("Homing is done\n");
                }
                break;
        }
    } 
    

    // 1ms의 모터 제어
    if(is_operational){
        // set target pos_tick
        motor1.setTrajectory(tick);

        // get pos_tick
        float pos_tick = motor1.getPosTick();

        // write a target velocity
        EC_WRITE_U32(domain1_pd + offset_target_position, pos_tick);

        // debugging
        // cout << "pos: " << EC_READ_S32(domain1_pd + offset_position_actual_value) << endl;
        // cout << "vel: " << EC_READ_S32(domain1_pd + offset_velocity_actual_value) << endl;
    } 

    if(is_logging){
        // logging
        motor1.logging(tick, EC_READ_S32(domain1_pd + offset_velocity_actual_value), EC_READ_S32(domain1_pd + offset_position_actual_value));
        tick += 0.001; 
        idx++;
    }

    // stop the motor
    if(is_stop){        
        EC_WRITE_U32(domain1_pd + offset_target_position, 0);
        if(EC_READ_S32(domain1_pd + offset_velocity_actual_value) == 0){
            printf("motor is stopped.\n");
            is_operational = 0;
            is_logging = 0;
            is_stop = 0;
            is_shutdown = 1;
        } 
    }

    // init the motor controller: switch on disabled
    if(is_shutdown){
        EC_WRITE_U16(domain1_pd + offset_control_word, 0x0000);
        if(((EC_READ_U16(domain1_pd + offset_status_word)) & 0x0040) == 0x0040){
            printf("operation enabled -> switch on disabled.\n");
            is_operational = 0;
            is_logging = 0;
            is_shutdown = 0;
            is_terminate = 1;
        }
    }

    // (마스터가) send process data
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);
}

/****************************************************************************/
char stopSignal;

// working in parallel with cyclic_task()
void *p_function(void *data){
    // get stop signal
    printf("Enter 'q' to stop motor\n");
    scanf("%c", &stopSignal);

    return nullptr;
};

/*****************************************************************************/

void stack_prefault(void)
{
    unsigned char dummy[MAX_SAFE_STACK];
    memset(dummy, 0, MAX_SAFE_STACK);
}

/****************************************************************************/

int main(int argc, char **argv)
{
    ec_slave_config_t *sc;
    struct timespec wakeup_time;
    int ret = 0;

    master = ecrt_request_master(0);
    if (!master) 
    {
        fprintf(stderr, "Failed to create master.\n");
        return -1;
    }

    domain1 = ecrt_master_create_domain(master);
    if (!domain1) 
    {
        fprintf(stderr, "Failed to create domain.\n");
        return -1;
    }

    if (!(slave_config = ecrt_master_slave_config(master, 0, 0, MAXON_EPOS4_5A))) 
    {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }

    printf("Configuring PDOs...\n");
    if (ecrt_slave_config_pdos(slave_config, EC_END, maxon_epos4_syncs_csp)) 
    {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) 
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    printf("Activating master...\n");
    if (ecrt_master_activate(master)) {
        return -1;
    }

    printf("Activating domain...\n");
    if (!(domain1_pd = ecrt_domain_data(domain1))) {
        return -1;
    }

    /* Set priority */
    struct sched_param param = {};
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);

    printf("Using priority %i.\n", param.sched_priority);
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        perror("sched_setscheduler failed");
    }
    
    // open shared memory
    int shm_fd = shm_open(SHARED_MEMORY_NAME, O_RDWR, 0666);
    while(shm_fd == -1){
        cout << "ROS node is not ready ..." << endl;
        usleep(10000);
        shm_fd = shm_open(SHARED_MEMORY_NAME, O_RDWR, 0666);
    }
    cout << "Succeed to open shared memory" << endl;


    // memory mapping
    shared_memory = static_cast<SharedMemoryData *>(mmap(
        nullptr, sizeof(SharedMemoryData), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0));
    if (shared_memory == MAP_FAILED)
    {
        cout << "Failed to map shared memory." << endl;
    }

    /* Lock memory */
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        fprintf(stderr, "Warning: Failed to lock memory: %s\n",
                strerror(errno));
    }

    stack_prefault();

    printf("Starting RT task with dt=%u ns.\n", PERIOD_NS);

    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
    wakeup_time.tv_sec += 1; /* start in future */
    wakeup_time.tv_nsec = 0;

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

        cyclic_task_csp();

        if(stopSignal == 'q'){
            printf("@@ Emergency Stop @@\n");
            stopSignal = '\0';
            is_stop = 1;
            is_operational = 0;
            is_logging = 0;
        }

        if(is_terminate == 1){
            printf("Terminate the program. Bye~ \n");
            break;
        }

        wakeup_time.tv_nsec += PERIOD_NS;
        while (wakeup_time.tv_nsec >= NSEC_PER_SEC) {
            wakeup_time.tv_nsec -= NSEC_PER_SEC;
            wakeup_time.tv_sec++;
        }
    }

    motor1.saveData(basePath + "csp_predict_pos02.txt", basePath + "csp_predict_vel02.txt", basePath + "csp_predict_acc02.txt", basePath + "csp_predict_jerk02.txt");

    // 공유 메모리 해제
    munmap(shared_memory, sizeof(SharedMemoryData));
    close(shm_fd);

    return ret;
}
/****************************************************************************/

