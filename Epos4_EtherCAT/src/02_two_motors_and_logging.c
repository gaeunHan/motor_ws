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
#define EPOSNUM 2

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
static unsigned int offset_control_word[EPOSNUM];
static unsigned int offset_target_velocity[EPOSNUM];
static unsigned int offset_velocity_offset[EPOSNUM];
static unsigned int offset_modes_of_operation[EPOSNUM];
static unsigned int offset_digital_outputs[EPOSNUM];
    

// TxPDO (slave -> master) offsets for PDO entries 
static unsigned int offset_status_word[EPOSNUM];
static unsigned int offset_position_actual_value[EPOSNUM];
static unsigned int offset_velocity_actual_value[EPOSNUM];
static unsigned int offset_torque_actual_value[EPOSNUM];
static unsigned int offset_modes_of_operation_display[EPOSNUM];
static unsigned int offset_digital_inputs[EPOSNUM];
    

// MDP Module CSV
const static ec_pdo_entry_reg_t domain1_regs[] = 
{
    // RxPDO
    {0,0, MAXON_EPOS4_5A, 0x6040, 0x00, &offset_control_word[0]},
    {0,0, MAXON_EPOS4_5A, 0x60FF, 0x00, &offset_target_velocity[0]},
    {0,0, MAXON_EPOS4_5A, 0x60B1, 0x00, &offset_velocity_offset[0]},
    {0,0, MAXON_EPOS4_5A, 0x6060, 0x00, &offset_modes_of_operation[0]},
    {0,0, MAXON_EPOS4_5A, 0x60FE, 0x01, &offset_digital_outputs[0]},

    // TxPDO
    {0,0, MAXON_EPOS4_5A, 0x6041, 0x00, &offset_status_word[0]},
    {0,0, MAXON_EPOS4_5A, 0x6064, 0x00, &offset_position_actual_value[0]},
    {0,0, MAXON_EPOS4_5A, 0x606C, 0x00, &offset_velocity_actual_value[0]},
    {0,0, MAXON_EPOS4_5A, 0x6077, 0x00, &offset_torque_actual_value[0]},
    {0,0, MAXON_EPOS4_5A, 0x6061, 0x00, &offset_modes_of_operation_display[0]},
    {0,0, MAXON_EPOS4_5A, 0x60FD, 0x00, &offset_digital_inputs[0]},

    // RxPDO
    {0,1, MAXON_EPOS4_5A, 0x6040, 0x00, &offset_control_word[1]},
    {0,1, MAXON_EPOS4_5A, 0x60FF, 0x00, &offset_target_velocity[1]},
    {0,1, MAXON_EPOS4_5A, 0x60B1, 0x00, &offset_velocity_offset[1]},
    {0,1, MAXON_EPOS4_5A, 0x6060, 0x00, &offset_modes_of_operation[1]},
    {0,1, MAXON_EPOS4_5A, 0x60FE, 0x01, &offset_digital_outputs[1]},

    // TxPDO
    {0,1, MAXON_EPOS4_5A, 0x6041, 0x00, &offset_status_word[1]},
    {0,1, MAXON_EPOS4_5A, 0x6064, 0x00, &offset_position_actual_value[1]},
    {0,1, MAXON_EPOS4_5A, 0x606C, 0x00, &offset_velocity_actual_value[1]},
    {0,1, MAXON_EPOS4_5A, 0x6077, 0x00, &offset_torque_actual_value[1]},
    {0,1, MAXON_EPOS4_5A, 0x6061, 0x00, &offset_modes_of_operation_display[1]},
    {0,1, MAXON_EPOS4_5A, 0x60FD, 0x00, &offset_digital_inputs[1]}
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
// logging vars.
# define PI 3.14159265
int T[EPOSNUM] = {1, 4}; // period of sine func in [sec]
int N = 4; // number of period to observe
int A[EPOSNUM] = {4000, 2000}; // amplitude of sine func
float t = 0;
int idx = 0;
float *t1_array, *t2_array;
uint32_t *vel1_input_array, *vel2_input_array;
uint32_t *vel1_output_array, *vel2_output_array;
int max(int a, int b){
    if(a >= b) return a;
    else return b;
}


/*****************************************************************************/
// cyclic task vars.
uint16_t status_word[EPOSNUM] = {0, 0};
uint16_t check_status_word[EPOSNUM];
uint16_t control_word[EPOSNUM] = {0, 0};
uint32_t target_velocity[EPOSNUM] = {0, 0};
bool is_operational[EPOSNUM] = {0, 0};

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
        status_word[0] = EC_READ_U16(domain1_pd + offset_status_word[0]);
        status_word[1] = EC_READ_U16(domain1_pd + offset_status_word[1]);
        
        // write process data on slave 1
        check_status_word[0] = status_word[0] & 0x006F; // check 0,1,2,3,5,6th bit only
        switch(check_status_word[0]){
            case 0x0040: // switch on disabled
                EC_WRITE_U16(domain1_pd + offset_control_word[0], 0x0006);
                printf("switch on disabled -> ready to switch on\n");
                break;

            case 0x0021: // ready to switch on
                EC_WRITE_U16(domain1_pd + offset_control_word[0], 0x0007);
                printf("ready to switch on -> switched on\n");
                break;

            case 0x0023: // switched on
                EC_WRITE_U16(domain1_pd + offset_control_word[0], 0x000F);
                printf("switched on -> operation enabled\n");
                break;

            case 0x0027: // operation enabled
                is_operational[0] = 1;
                EC_WRITE_U16(domain1_pd + offset_modes_of_operation[0], 9); // select csv mode
                //EC_WRITE_U32(domain1_pd + offset_target_velocity[0], target_velocity[0]);
                break;

            case 0x0008: //fault
                printf("fault\n");

                // get controlword
                control_word[0] = EC_READ_U16(domain1_pd + offset_control_word[0]);

                // fault reset
                control_word[0] |= 0x0080; 
                EC_WRITE_U16(domain1_pd + offset_control_word[0], control_word[0]);
                break;
        }

        // write process data on slave 2
        check_status_word[1] = status_word[1] & 0x006F; // check 0,1,2,3,5,6th bit only
        switch(check_status_word[1]){
            case 0x0040: // switch on disabled
                EC_WRITE_U16(domain1_pd + offset_control_word[1], 0x0006);
                printf("switch on disabled -> ready to switch on\n");
                break;

            case 0x0021: // ready to switch on
                EC_WRITE_U16(domain1_pd + offset_control_word[1], 0x0007);
                printf("ready to switch on -> switched on\n");
                break;

            case 0x0023: // switched on
                EC_WRITE_U16(domain1_pd + offset_control_word[1], 0x000F);
                printf("switched on -> operation enabled\n");
                break;

            case 0x0027: // operation enabled
                is_operational[1] = 1;
                EC_WRITE_U16(domain1_pd + offset_modes_of_operation[1], 9); // select csv mode
                //EC_WRITE_U32(domain1_pd + offset_target_velocity[1], target_velocity[1]);
                break;

            case 0x0008: //fault
                printf("fault\n");

                // get controlword
                control_word[1] = EC_READ_U16(domain1_pd + offset_control_word[1]);

                // fault reset
                control_word[1] |= 0x0080; 
                EC_WRITE_U16(domain1_pd + offset_control_word[1], control_word[1]);
                break;
        }
    } 

    // create sine velocity profile and log datum
    if(is_operational[0] * is_operational[1]){
        // target velocity for slave 1
        target_velocity[0] = A[0]*sin((2*PI/T[0])*t);
        EC_WRITE_U32(domain1_pd + offset_target_velocity[0], target_velocity[0]);

        // target velocity for slave 2
        target_velocity[1] = A[1]*sin((2*PI/T[1])*t);
        EC_WRITE_U32(domain1_pd + offset_target_velocity[1], target_velocity[1]);

        // logging
        t1_array[idx] = t;
        vel1_input_array[idx] = target_velocity[0];
        vel1_output_array[idx] = EC_READ_S32(domain1_pd + offset_velocity_actual_value[0]); // Read a 32-bit signed value from EtherCAT data.
        t2_array[idx] = t;
        vel2_input_array[idx] = target_velocity[1];
        vel2_output_array[idx] = EC_READ_S32(domain1_pd + offset_velocity_actual_value[1]);
        t += 0.001; 
        idx++;
    } 

    // (마스터가) send process data
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);
}

/****************************************************************************/
char stopSignal;

// working in parallel with cyclic_task()
void *p_function(void *data){
    printf("Enter 'q' to stop motor\n");
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

    // register the first slave
    if (!(slave_config = ecrt_master_slave_config(master, 0, 0, MAXON_EPOS4_5A))) 
    {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }

    // configure PDO of the first slave
    printf("Configuring PDOs...\n");
    if (ecrt_slave_config_pdos(slave_config, EC_END, maxon_epos4_syncs_csv)) 
    {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

    // register the second slave
    if (!(slave_config = ecrt_master_slave_config(master, 0, 1, MAXON_EPOS4_5A))) 
    {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }

    // configure PDO of the second slave
    printf("Configuring PDOs...\n");
    if (ecrt_slave_config_pdos(slave_config, EC_END, maxon_epos4_syncs_csv)) 
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
    t1_array = (float *)malloc((N * max(T[0],T[1]) * 1000) * sizeof(float));
    vel1_input_array = (uint32_t *)malloc((N * max(T[0],T[1]) * 1000) * sizeof(uint32_t));
    vel1_output_array = (uint32_t *)malloc((N * max(T[0],T[1])* 1000) * sizeof(uint32_t));
    t2_array = (float *)malloc((N * max(T[0],T[1]) * 1000) * sizeof(float));
    vel2_input_array = (uint32_t *)malloc((N * max(T[0],T[1]) * 1000) * sizeof(uint32_t));
    vel2_output_array = (uint32_t *)malloc((N * max(T[0],T[1])* 1000) * sizeof(uint32_t));


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

        if(stopSignal=='q' || idx > N*max(T[0],T[1])*1000) break;

        wakeup_time.tv_nsec += PERIOD_NS;
        while (wakeup_time.tv_nsec >= NSEC_PER_SEC) {
            wakeup_time.tv_nsec -= NSEC_PER_SEC;
            wakeup_time.tv_sec++;
        }
    }

    FILE* file1 = fopen("slave1_data.txt", "w");
    FILE* file2 = fopen("slave2_data.txt", "w");

    if(file1 != NULL){
        for(int i=0; i < N*max(T[0],T[1])*1000; i++){
            fprintf(file1, "%f %d %d\n", t1_array[i], vel1_input_array[i], vel1_output_array[i]);
            fprintf(file2, "%f %d %d\n", t1_array[i], vel2_input_array[i], vel2_output_array[i]);
        }
    }

    fclose(file1);
    fclose(file2);
    printf("values saved to file. \n");
    printf("slave 1: \n\tT: %d, N: %d, A: %d\n", T[0], N, A[0]);
    printf("slave 2: \n\tT: %d, N: %d, A: %d\n", T[1], N, A[1]);

    free(t1_array);
    free(vel1_input_array);
    free(vel1_output_array);
    free(t2_array);
    free(vel2_input_array);
    free(vel2_output_array);

    return ret;
}
/****************************************************************************/

