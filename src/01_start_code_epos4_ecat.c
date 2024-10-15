// EPOS4를 이용하여(하나의 slave) PPM(Profile Position Mode)으로 위치 제어를 해보자. 

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
#include <sched.h> /* sched_setscheduler() */
#include <pthread.h>
#include <math.h>

/****************************************************************************/

#include "ecrt.h"
#include "motor.h"

/****************************************************************************/

/** Task period in ns. */
#define PERIOD_NS   (1000000)

#define MAX_SAFE_STACK (8 * 1024) /* The maximum stack size which is
                                     guranteed safe to access without
                                     faulting */

/****************************************************************************/

/* Constants */
#define NSEC_PER_SEC (1000000000)
#define FREQUENCY (NSEC_PER_SEC / PERIOD_NS)

/****************************************************************************/

// EtherCAT
// 마스터 오브젝트(master)가 마스터 장치를 의미한다면, 도메인(domain)은 주기적으로 통신할 데이터(PD)의 메모리 영역을 의미한다.
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

static ec_slave_config_t *slave_config = NULL;
static ec_slave_config_state_t slave_config_state = {};
/****************************************************************************/

// process data
static uint8_t *domain1_pd = NULL; // for PPM
static uint8_t *domain2_pd = NULL; // for CSV

#define MAXON_EPOS4_5A 0x000000fb, 0x61500000 // Product Number 확인 필요(ESI file)

static unsigned int counter = 0;


// RxPDO (master -> slave) offsets for PDO entries
    /* MDP module for PPM */
    static unsigned int offset_control_word;
    static unsigned int offset_target_postion;
    static unsigned int offset_profile_acceleration;
    static unsigned int offset_profile_deceleration;
    static unsigned int offset_profile_velocity;
    static unsigned int offset_modes_of_operation;
    static unsigned int offset_digital_outputs;
    /* MDP module for CSV */
    static unsigned int offset_target_velocity;
    static unsigned int offset_velocity_offset;

// TxPDO (slave -> master) offsets for PDO entries 
    /* MDP module for PPM */ 
    static unsigned int offset_status_word;
    static unsigned int offset_position_actual_value;
    static unsigned int offset_velocity_actual_value;
    static unsigned int offset_following_error_actual_value;
    static unsigned int offset_modes_of_operation_display;
    static unsigned int offset_digital_inputs;
    /* MDP module for CSV */
    static unsigned int offset_torque_actual_value;

// MDP Module PPM
const static ec_pdo_entry_reg_t domain1_regs[] = 
{
    // RxPDO
    {0,0, MAXON_EPOS4_5A, 0x6040, 0x00, &offset_control_word},
    {0,0, MAXON_EPOS4_5A, 0x607A, 0x00, &offset_target_postion},
    {0,0, MAXON_EPOS4_5A, 0x6083, 0x00, &offset_profile_acceleration},
    {0,0, MAXON_EPOS4_5A, 0x6084, 0x00, &offset_profile_deceleration},
    {0,0, MAXON_EPOS4_5A, 0x6081, 0x00, &offset_profile_velocity},
    {0,0, MAXON_EPOS4_5A, 0x6060, 0x00, &offset_modes_of_operation},
    {0,0, MAXON_EPOS4_5A, 0x60FE, 0x01, &offset_digital_outputs},

    // TxPDO
    {0,0, MAXON_EPOS4_5A, 0x6041, 0x00, &offset_status_word},
    {0,0, MAXON_EPOS4_5A, 0x6064, 0x00, &offset_position_actual_value},
    {0,0, MAXON_EPOS4_5A, 0x606C, 0x00, &offset_velocity_actual_value},
    {0,0, MAXON_EPOS4_5A, 0x60F4, 0x00, &offset_following_error_actual_value},
    {0,0, MAXON_EPOS4_5A, 0x6061, 0x00, &offset_modes_of_operation_display},
    {0,0, MAXON_EPOS4_5A, 0x60FD, 0x00, &offset_digital_inputs}
};

// MDP Module CSV
const static ec_pdo_entry_reg_t domain2_regs[] = 
{
    // RxPDO
    {0,0, MAXON_EPOS4_5A, 0x6040, 0x00, &offset_control_word},
    {0,0, MAXON_EPOS4_5A, 0x60FF, 0x00, &offset_target_velocity},
    {0,0, MAXON_EPOS4_5A, 0x60B1, 0x00, &offset_velocity_offset},
    {0,0, MAXON_EPOS4_5A, 0x6060, 0x00, &offset_modes_of_operation},
    {0,0, MAXON_EPOS4_5A, 0x60FE, 0x01, &offset_digital_outputs},

    // TxPDO
    {0,0, MAXON_EPOS4_5A, 0x6041, 0x00, &offset_status_word},
    {0,0, MAXON_EPOS4_5A, 0x6064, 0x00, &offset_position_actual_value},
    {0,0, MAXON_EPOS4_5A, 0x606C, 0x00, &offset_velocity_actual_value},
    {0,0, MAXON_EPOS4_5A, 0x6077, 0x00, &offset_torque_actual_value},
    {0,0, MAXON_EPOS4_5A, 0x6061, 0x00, &offset_modes_of_operation_display},
    {0,0, MAXON_EPOS4_5A, 0x60FD, 0x00, &offset_digital_inputs}
};



/**************************** MDP module PPM mapping ****************************/
static	ec_pdo_entry_info_t maxon_epos4_pdo_entries[] = 
{
    // RxPDO (Master -> Slave)
    {0x6040, 0x00, 16},    // control word
    {0x607A, 0x00, 32},    // target position
    {0x6083, 0x00, 32},    // profile acceleration
    {0x6084, 0x00, 32},    // profile deceleration
    {0x6081, 0x00, 32},    // profile velocity
    {0x6060, 0x00, 8},     // modes of operation
    {0x60FE, 0x01, 32},    // digital outputs

    // TxPDO (Slave -> Master)
    {0x6041, 0x00, 16},    // status word
    {0x6064, 0x00, 32},    // position actual value
    {0x606C, 0x00, 32},    // velocity actual value
    {0x60F4, 0x00, 32},    // following error actual value
    {0x6061, 0x00, 8},     // modes of operation display
    {0x60FD, 0x00, 32}    // digital inputs
};

// Enable PDO by writing the value of the number of objects in object
static	ec_pdo_info_t maxon_epos4_pdos[] = 
{
    // RxPDO(Master -> Slave) 1 mapping
    {0x1600, 7,	maxon_epos4_pdo_entries + 0}, // 7개의 RxPDO entry를 mapping 할 것인데, entry의 시작 위치는 maxon_epos4_pdo_entries[0]이다. 

    // TxPDO(Master <- Slave) 1 mapping
    {0x1a00, 6,	maxon_epos4_pdo_entries + 7} // 6개의 TxPDO entry를 mapping 할 것인데, entry의 시작 위치는 maxon_epos4_pdo_entries[7]이다. 
    
};

// slave sync manager
// EC_DIR_OUTPUT: Master -> Slave, EC_DIR_INPUT: Master <- Slave
static ec_sync_info_t maxon_epos4_syncs[] = {
	{ 0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE },
	{ 1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE },
	{ 2, EC_DIR_OUTPUT, 1, maxon_epos4_pdos + 0, EC_WD_ENABLE },
	{ 3, EC_DIR_INPUT,  1, maxon_epos4_pdos + 1, EC_WD_DISABLE },
	{ 0xff }
};

/**************************** MDP module CSV mapping ****************************/
static ec_pdo_entry_info_t csv_pdo_entries[] = {
    // RxPDO (Master -> Slave)
    {0x6040, 0x00, 16},    // control word
    {0x60FF, 0x00, 32},    // target velocity
    {0x60B1, 0x00, 32},    // velocity offset
    {0x6060, 0x00, 8},     // modes of operation
    {0x60FE, 0x01, 32},    // digital outputs

    // TxPDO (Slave -> Master)
    {0x6041, 0x00, 16},    // status word
    {0x6064, 0x00, 32},    // position actual value
    {0x606C, 0x00, 32},    // velocity actual value
    {0x6077, 0x00, 16},    // torque actual value
    {0x6061, 0x00, 8},     // modes of operation display
    {0x60FD, 0x00, 32}     // digital inputs
};

static ec_pdo_info_t csv_pdos[] = {
    // RxPDO(Master -> Slave) 1 mapping
    {0x1600, 5,	csv_pdo_entries + 0}, // 5개의 RxPDO entry를 mapping 할 것인데, entry의 시작 위치는 maxon_epos4_pdo_entries[0]이다. 

    // TxPDO(Master <- Slave) 1 mapping
    {0x1a00, 6,	csv_pdo_entries + 5} // 6개의 TxPDO entry를 mapping 할 것인데, entry의 시작 위치는 maxon_epos4_pdo_entries[5]이다. 
};

// slave sync manager
// EC_DIR_OUTPUT: Master -> Slave, EC_DIR_INPUT: Master <- Slave
static ec_sync_info_t maxon_epos4_syncs_csv[] = {
	{ 0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE },
	{ 1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE },
	{ 2, EC_DIR_OUTPUT, 1, csv_pdos + 0, EC_WD_ENABLE },
	{ 3, EC_DIR_INPUT,  1, csv_pdos + 1, EC_WD_DISABLE },
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

uint16_t status_word = 0;
uint16_t prev_status_word = -1;
uint16_t check_status_word;
uint16_t control_word = 0;
/* statusword bit */
uint16_t setPointAcknowledge = 0b0001000000000000;
/* controlword bit */
uint16_t set_rel_target_pos = 0b0000000001000000;
uint16_t set_abs_target_pos = 0b1111111110111111;
uint16_t set_0_newSetpoint = 0b1111111111101111;
uint16_t set_1_newSetpoint = 0b0000000000010000;
int target_position_cnt = 0;
int target_velocity = 0;

//밑에 while 문에서 cyclic_task_ppm이 계속 돌고 있다(1000Hz, 1ms 주기)  
void cyclic_task_ppm()
{
    // (마스터가) receive process data
    ecrt_master_receive(master); // 데이터 받아(queue에서 데이터그램 dispatch)
    ecrt_domain_process(domain1); // 정상적인 통신 이루어졌는지 확인, 도메인 상태 업뎃

    // check process data state - optional
    check_domain1_state(); // ecrt_domain_state(), 도메인 상태 확인 

    /*
        if-else state: 주기적인 작업을 수행하기 위한 타이머 역할. 
        counter가 0이 될때마다 else문의 작업 수행. 
        즉, 1ms의 real-time으로 동작시키는 cyclic_task 중, 
        실시간성이 필요하지 않은 작업들은 1s 단위의 else문에 넣어 성능 최적화(전력소모, CPU 사용량 등). 
    */
    if (counter)  
    {
        counter--;   
    } 
    else
    { 
        /* do this at 1 Hz, 1 sec */  
        counter = FREQUENCY;  
        
        // check for master state
        check_master_state(); // ecrt_master_state()

        // check for slave configuration state(s)
        check_slave_config_states(); // ecrt_slave_config_state()

        // get statusword
        status_word = EC_READ_U16(domain1_pd + offset_status_word);

        // // print status word if changed
        // if(status_word != prev_status_word){        
        //     printf("current statusword: 0x%04X\n\n", status_word);
        //     prev_status_word = status_word;
        // }
        
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
                //printf("operation enabled reached\n");

                // get controlword
                control_word = EC_READ_U16(domain1_pd + offset_control_word);

                // set position mode(abs/rel)
                control_word |= set_rel_target_pos;

                // set target position
                EC_WRITE_U16(domain1_pd + offset_target_postion, target_position_cnt);

                // determine new setpoint bit to give new target position
                if((status_word & setPointAcknowledge) == setPointAcknowledge){
                    control_word &= set_0_newSetpoint;
                    //printf("1 -> 0\n");
                } 
                else{
                    control_word |= set_1_newSetpoint; 
                    //printf("0 -> 1\n");
                } 
                // write new setpoint bit on controlword
                EC_WRITE_U16(domain1_pd + offset_control_word, control_word);
                break;

            case 0x0008: //fault
                printf("fault\n");

                // get controlword
                control_word = EC_READ_U16(domain1_pd + offset_control_word);

                // fault reset
                control_word |= 0x0080; 
                EC_WRITE_U16(domain1_pd + offset_control_word, control_word);
                break;
        }

        // // debugging
        // printf("~~ current statusword: 0x%04X\n", status_word);
        // printf("~~ current controlword: 0x%04X\n", EC_READ_U16(domain1_pd + offset_control_word));
        // printf("~~ Target position: %d\n\n", EC_READ_U16(domain1_pd + offset_target_postion));

    } 

    // (마스터가) send process data
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);
}

// 1ms period
void cyclic_task_csv()
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
    else // 1s period
    { 
        counter = FREQUENCY;  
        
        // check for master state
        check_master_state(); // ecrt_master_state()

        // check for slave configuration state(s)
        check_slave_config_states(); // ecrt_slave_config_state()

        // get statusword
        status_word = EC_READ_U16(domain2_pd + offset_status_word);
        
        // write process data   
        check_status_word = status_word & 0x006F; // check 0,1,2,3,5,6th bit only
        switch(check_status_word){
            case 0x0040: // switch on disabled
                EC_WRITE_U16(domain2_pd + offset_control_word, 0x0006);
                printf("switch on disabled -> ready to switch on\n");
                break;

            case 0x0021: // ready to switch on
                EC_WRITE_U16(domain2_pd + offset_control_word, 0x0007);
                printf("ready to switch on -> switched on\n");
                break;

            case 0x0023: // switched on
                EC_WRITE_U16(domain2_pd + offset_control_word, 0x000F);
                printf("switched on -> operation enabled\n");
                break;

            case 0x0027: // operation enabled
                EC_WRITE_U16(domain2_pd + offset_modes_of_operation, 9); // select csv mode
                EC_WRITE_U32(domain2_pd + offset_target_velocity, target_velocity);
                break;

            case 0x0008: //fault
                printf("fault\n");

                // get controlword
                control_word = EC_READ_U16(domain2_pd + offset_control_word);

                // fault reset
                control_word |= 0x0080; 
                EC_WRITE_U16(domain2_pd + offset_control_word, control_word);
                break;
        }

        // // debugging
        // printf("~~ current statusword: 0x%04X\n", status_word);
        // printf("~~ current controlword: 0x%04X\n", EC_READ_U16(domain1_pd + offset_control_word));
        // printf("~~ Target position: %d\n\n", EC_READ_U16(domain1_pd + offset_target_postion));

    } 

    // (마스터가) send process data
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);
}

/****************************************************************************/

// working in parallel with cyclic_task()
void *p_function(void *data){
    while(1){
        // create a motor obj
        struct Motor motor1;
        MotorID(&motor1, 1);
        SetEncodorandGear(&motor1, 1024, 1);
        EncodorHomeposition(&motor1, 0);
        SetJointMinMiax(&motor1, -90, 90);

        // // ppm - get degree to move motor
        // int degree;
        // printf("Enter a target position in degree: ");
        // scanf("%d", &degree);
        // SetDegree(&motor1, degree);
        // target_position_cnt = GetEncoderCnt(&motor1);

        // csv - get target velocity
        printf("Enter a target velocity: ");
        scanf("%d", &target_velocity);

        sleep(1);
    }    
}

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
    if (ecrt_slave_config_pdos(slave_config, EC_END, maxon_epos4_syncs_csv)) 
    {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

    if (ecrt_domain_reg_pdo_entry_list(domain1, domain2_regs)) 
    {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    printf("Activating master...\n");
    if (ecrt_master_activate(master)) {
        return -1;
    }

    printf("Activating domain...\n");
    if (!(domain2_pd = ecrt_domain_data(domain1))) {
        return -1;
    }

    /* Set priority */
    struct sched_param param = {};
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);

    printf("Using priority %i.", param.sched_priority);
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        perror("sched_setscheduler failed");
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

    thr_id = pthread_create(&pthread, NULL, p_function, (void*)p1); //2
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

        //cyclic_task_ppm();
        cyclic_task_csv();

        wakeup_time.tv_nsec += PERIOD_NS;
        while (wakeup_time.tv_nsec >= NSEC_PER_SEC) {
            wakeup_time.tv_nsec -= NSEC_PER_SEC;
            wakeup_time.tv_sec++;
        }
    }

    return ret;
}
/****************************************************************************/
