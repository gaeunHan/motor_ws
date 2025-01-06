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
#include <stdbool.h> /* boolean */

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
static uint8_t *domain1_pd = NULL; 

#define MAXON_EPOS4_5A 0x000000fb, 0x61500000 // Product Number 확인 필요(ESI file)

static unsigned int counter = 0;


// RxPDO (master -> slave) offsets for PDO entries
static unsigned int offset_control_word;
static unsigned int offset_target_position;
static unsigned int offset_position_offset;
static unsigned int offset_torque_offset;
static unsigned int offset_modes_of_operation;
static unsigned int offset_digital_outputs;
    

// TxPDO (slave -> master) offsets for PDO entries 
static unsigned int offset_status_word;
static unsigned int offset_position_actual_value;
static unsigned int offset_velocity_actual_value;
static unsigned int offset_torque_actual_value;
static unsigned int offset_modes_of_operation_display;
static unsigned int offset_digital_inputs;
static unsigned int offset_error_code;
    

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

    // TxPDO
    {0,0, MAXON_EPOS4_5A, 0x6041, 0x00, &offset_status_word},
    {0,0, MAXON_EPOS4_5A, 0x6064, 0x00, &offset_position_actual_value},
    {0,0, MAXON_EPOS4_5A, 0x606C, 0x00, &offset_velocity_actual_value},
    {0,0, MAXON_EPOS4_5A, 0x6077, 0x00, &offset_torque_actual_value},
    {0,0, MAXON_EPOS4_5A, 0x6061, 0x00, &offset_modes_of_operation_display},
    {0,0, MAXON_EPOS4_5A, 0x60FD, 0x00, &offset_digital_inputs},
    {0,0, MAXON_EPOS4_5A, 0x603F, 0x00, &offset_error_code}
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

    // TxPDO (Slave -> Master)
    {0x6041, 0x00, 16},    // status word
    {0x6064, 0x00, 32},    // position actual value
    {0x606C, 0x00, 32},    // velocity actual value
    {0x6077, 0x00, 16},    // torque actual value
    {0x6061, 0x00, 8},     // modes of operation display
    {0x60FD, 0x00, 32},    // digital inputs
    {0x603F, 0x00, 16}     // error code
};

static ec_pdo_info_t csp_pdos[] = {
    // RxPDO(Master -> Slave) 1 mapping
    {0x1600, 6,	csp_pdo_entries + 0}, // 6개의 RxPDO entry를 mapping 할 것인데, entry의 시작 위치는 maxon_epos4_pdo_entries[0]이다. 

    // TxPDO(Master <- Slave) 1 mapping
    {0x1a00, 7,	csp_pdo_entries + 6} // 7개의 TxPDO entry를 mapping 할 것인데, entry의 시작 위치는 maxon_epos4_pdo_entries[6]이다. 
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
// logging vars.
float t = 0;
int idx = 0;
float *t1_array;
float *velocity_input_array;
float *position_input_array;
int32_t *velocity_output_array;
float *position_output_array;

/*****************************************************************************/
// trajectory generator
#define CNT_PER_DEGREE 398.0 // 1024*4*35/360
float pos[2] = {0.0, 180.0*CNT_PER_DEGREE};
float vel[2] = {0.0, 0.0};
float acc[2] = {0.0, 0.0};
float moveTime[2] = {0.0, 1.0};
float currTime;
float pos_t, vel_t, acc_t;

void getTrajectory(float q0, float q1, float v0, float v1, float a0, float a1, float t0, float t1){
    // calc coefficients
    float b0, b1, b2, b3, b4, b5;
    float T = t1 - t0;
    b0 = q0;
    b1 = v0;
    b2 = 0.5 * a0;
    b3 = (1.0 / (2 * T * T * T)) * (20 * (q1 - q0) - (8 * v1 + 12 * v0) * T - (3 * a0 - a1) * T * T);
    b4 = (1.0 / (2 * T * T * T * T)) * (-30 * (q1 - q0) + (14 * v1 + 16 * v0) * T + (3 * a0 - 2 * a1) * T * T);
    b5 = (1.0 / (2 * T * T * T * T * T)) * (12 * (q1 - q0) - 6 * (v1 + v0) * T + (a1 - a0) * T * T);

    // pos, vel, acc formula
    currTime = t;
    float dt = currTime - t0;
    pos_t = b0 + b1*dt + b2*pow(dt,2) + b3*pow(dt,3) + b4*pow(dt,4) + b5*pow(dt,5); // [encoder cnt]
    vel_t = b1 + 2*b2*dt + 3*b3*pow(dt,2) + 4*b4*pow(dt,3) + 5*b5*pow(dt,4); // [encoder cnt / sec]
    acc_t = 2*b2 + 6*b3*dt + 12*b4*pow(dt,2) + 20*b5*pow(dt,3);

}

/*****************************************************************************/
// cyclic task vars.
uint16_t status_word = 0;
uint16_t prev_status_word = -1;
uint16_t check_status_word;
uint16_t control_word = 0;
uint16_t error_code = 0;
bool is_operational = 0;

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
                is_operational = 1;
                EC_WRITE_U16(domain1_pd + offset_modes_of_operation, 8); // select csp mode
                break;

            case 0x0008: //fault
                // read error code
                error_code = EC_READ_U16(domain1_pd + offset_error_code);
                printf("Fault, error code is: 0x%04X\n", error_code);

                // get controlword
                control_word = EC_READ_U16(domain1_pd + offset_control_word);

                // fault reset
                control_word |= 0x0080; 
                EC_WRITE_U16(domain1_pd + offset_control_word, control_word);

                // following error handling
                if((error_code & 0xFFFF) == 0x8611){
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

    // apply PI control and log datum
    if(is_operational){
        // get target position by following 5th-poly trajectory
        getTrajectory(pos[0], pos[1], vel[0], vel[1], acc[0], acc[1], moveTime[0], moveTime[1]);

        // write a target position
        EC_WRITE_U32(domain1_pd + offset_target_position, pos_t);

        // logging
        t1_array[idx] = t;
        velocity_input_array[idx] = (vel_t / (4096.0 * 35.0)) * 60; // [rpm]
        position_input_array[idx] = pos_t * 360.0 / 4096.0 / 35.0; // [deg]
        velocity_output_array[idx] = (EC_READ_S32(domain1_pd + offset_velocity_actual_value))/35.0; // actual velocity 읽을 때 기어비로 나눠줘야 함. 
        position_output_array[idx] = (((float)EC_READ_S32(domain1_pd + offset_position_actual_value) * 360.0f) / 4096.0f) / 35.0; // logging pos in degree - *(360/4096)하면 0 되어버림
        t += 0.001; 
        idx++;
    } 

    // debugging
    // printf("curr_pos_cnt: %d\n", EC_READ_S32(domain1_pd + offset_position_actual_value));
    // printf("curr_pos_deg: %f\n\n", ((float)EC_READ_S32(domain1_pd + offset_position_actual_value) * 360.0f) / 4096.0f);
     
    // (마스터가) send process data
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);
}

/****************************************************************************/
char stopSignal;

// working in parallel with cyclic_task()
void *p_function(void *data){
    // get stop signal
    printf("Enter 'q' to stop motor");
    scanf("%c", &stopSignal);
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

    thr_id = pthread_create(&pthread, NULL, p_function, (void*)p1);
    if(thr_id < 0)
    {
        perror("pthread0 create error");
        exit(EXIT_FAILURE);
    }

    /* allocate memory for logging */
    t1_array = (float *)malloc((1000) * sizeof(float)); // t가 0.001씩 증가 -> 1000이면 되는데 걍 넉넉잡아 2000
    velocity_input_array = (float *)malloc((1000) * sizeof(float));
    position_input_array = (float *)malloc((1000) * sizeof(float));
    velocity_output_array = (uint32_t *)malloc((1000) * sizeof(int32_t));
    position_output_array = (float *)malloc((1000) * sizeof(float));

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

        cyclic_task_csv();

        if(stopSignal=='q' || idx > 1000) break;

        wakeup_time.tv_nsec += PERIOD_NS;
        while (wakeup_time.tv_nsec >= NSEC_PER_SEC) {
            wakeup_time.tv_nsec -= NSEC_PER_SEC;
            wakeup_time.tv_sec++;
        }
    }

    FILE* file1 = fopen("/home/ghan/study_ws/epos4_etherCAT/logging/5th_pos99.txt", "w");
    FILE* file2 = fopen("/home/ghan/study_ws/epos4_etherCAT/logging/5th_vel99.txt", "w");

    if(file1 != NULL && file2 != NULL){
        for(int i=0; i < 1000; i++){
            fprintf(file1, "%f %f %f\n", t1_array[i], position_input_array[i], position_output_array[i]);
            fprintf(file2, "%f %f %d\n", t1_array[i], velocity_input_array[i], velocity_output_array[i]);
        }
    }

    fclose(file1);
    fclose(file2);
    printf("values saved to file. \n");

    free(t1_array);
    free(velocity_output_array);
    free(position_output_array);

    return ret;
}
/****************************************************************************/

