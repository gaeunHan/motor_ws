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
#include <cmath>
#include <iostream>
#include <fstream> /* file */
#include <vector>
#include <string>
#include "ecrt.h"
#include "CSVMotorController.h"

using namespace std;

/****************************************************************************/
#define PERIOD_NS   (1000000) /** Task period in ns. */
#define MAX_SAFE_STACK (8 * 1024) /* The maximum stack size which is guranteed safe to access without faulting */
#define NSEC_PER_SEC (1000000000)
#define FREQUENCY (NSEC_PER_SEC / PERIOD_NS)
#define MAXON_EPOS4_5A 0x000000fb, 0x60500000 // Product Number 확인 필요(ESI file)
#define TARGET_NUM 5
#define MOTORNUM 8

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
static unsigned int offset_control_word[MOTORNUM];
static unsigned int offset_target_velocity[MOTORNUM];
static unsigned int offset_velocity_offset[MOTORNUM];
static unsigned int offset_modes_of_operation[MOTORNUM];
static unsigned int offset_digital_outputs[MOTORNUM];
static unsigned int offset_min_position_limit[MOTORNUM];
static unsigned int offset_max_position_limit[MOTORNUM];
    

// TxPDO (slave -> master) offsets for PDO entries 
static unsigned int offset_status_word[MOTORNUM];
static unsigned int offset_position_actual_value[MOTORNUM];
static unsigned int offset_velocity_actual_value[MOTORNUM];
static unsigned int offset_torque_actual_value[MOTORNUM];
static unsigned int offset_modes_of_operation_display[MOTORNUM];
static unsigned int offset_digital_inputs[MOTORNUM];
static unsigned int offset_epos4_error_code[MOTORNUM];
    

// MDP Module CSV
const static ec_pdo_entry_reg_t domain1_regs[] = 
{
    /* Motor 1 */
    // RxPDO
    {0,0, MAXON_EPOS4_5A, 0x6040, 0x00, &offset_control_word[0]},
    {0,0, MAXON_EPOS4_5A, 0x60FF, 0x00, &offset_target_velocity[0]},
    {0,0, MAXON_EPOS4_5A, 0x60B1, 0x00, &offset_velocity_offset[0]},
    {0,0, MAXON_EPOS4_5A, 0x6060, 0x00, &offset_modes_of_operation[0]},
    {0,0, MAXON_EPOS4_5A, 0x60FE, 0x01, &offset_digital_outputs[0]},
    {0,0, MAXON_EPOS4_5A, 0x607D, 0x01, &offset_min_position_limit[0]},
    {0,0, MAXON_EPOS4_5A, 0x607D, 0x02, &offset_max_position_limit[0]},
    // TxPDO
    {0,0, MAXON_EPOS4_5A, 0x6041, 0x00, &offset_status_word[0]},
    {0,0, MAXON_EPOS4_5A, 0x6064, 0x00, &offset_position_actual_value[0]},
    {0,0, MAXON_EPOS4_5A, 0x606C, 0x00, &offset_velocity_actual_value[0]},
    {0,0, MAXON_EPOS4_5A, 0x6077, 0x00, &offset_torque_actual_value[0]},
    {0,0, MAXON_EPOS4_5A, 0x6061, 0x00, &offset_modes_of_operation_display[0]},
    {0,0, MAXON_EPOS4_5A, 0x60FD, 0x00, &offset_digital_inputs[0]},
    {0,0, MAXON_EPOS4_5A, 0x603F, 0x00, &offset_epos4_error_code[0]},

    /* Motor 2 */
    // RxPDO
    {0,1, MAXON_EPOS4_5A, 0x6040, 0x00, &offset_control_word[1]},
    {0,1, MAXON_EPOS4_5A, 0x60FF, 0x00, &offset_target_velocity[1]},
    {0,1, MAXON_EPOS4_5A, 0x60B1, 0x00, &offset_velocity_offset[1]},
    {0,1, MAXON_EPOS4_5A, 0x6060, 0x00, &offset_modes_of_operation[1]},
    {0,1, MAXON_EPOS4_5A, 0x60FE, 0x01, &offset_digital_outputs[1]},
    {0,1, MAXON_EPOS4_5A, 0x607D, 0x01, &offset_min_position_limit[1]},
    {0,1, MAXON_EPOS4_5A, 0x607D, 0x02, &offset_max_position_limit[1]},
    // TxPDO
    {0,1, MAXON_EPOS4_5A, 0x6041, 0x00, &offset_status_word[1]},
    {0,1, MAXON_EPOS4_5A, 0x6064, 0x00, &offset_position_actual_value[1]},
    {0,1, MAXON_EPOS4_5A, 0x606C, 0x00, &offset_velocity_actual_value[1]},
    {0,1, MAXON_EPOS4_5A, 0x6077, 0x00, &offset_torque_actual_value[1]},
    {0,1, MAXON_EPOS4_5A, 0x6061, 0x00, &offset_modes_of_operation_display[1]},
    {0,1, MAXON_EPOS4_5A, 0x60FD, 0x00, &offset_digital_inputs[1]},
    {0,1, MAXON_EPOS4_5A, 0x603F, 0x00, &offset_epos4_error_code[1]},

    /* Motor 3 */
    // RxPDO
    {0,2, MAXON_EPOS4_5A, 0x6040, 0x00, &offset_control_word[2]},
    {0,2, MAXON_EPOS4_5A, 0x60FF, 0x00, &offset_target_velocity[2]},
    {0,2, MAXON_EPOS4_5A, 0x60B1, 0x00, &offset_velocity_offset[2]},
    {0,2, MAXON_EPOS4_5A, 0x6060, 0x00, &offset_modes_of_operation[2]},
    {0,2, MAXON_EPOS4_5A, 0x60FE, 0x01, &offset_digital_outputs[2]},
    {0,2, MAXON_EPOS4_5A, 0x607D, 0x01, &offset_min_position_limit[2]},
    {0,2, MAXON_EPOS4_5A, 0x607D, 0x02, &offset_max_position_limit[2]},
    // TxPDO
    {0,2, MAXON_EPOS4_5A, 0x6041, 0x00, &offset_status_word[2]},
    {0,2, MAXON_EPOS4_5A, 0x6064, 0x00, &offset_position_actual_value[2]},
    {0,2, MAXON_EPOS4_5A, 0x606C, 0x00, &offset_velocity_actual_value[2]},
    {0,2, MAXON_EPOS4_5A, 0x6077, 0x00, &offset_torque_actual_value[2]},
    {0,2, MAXON_EPOS4_5A, 0x6061, 0x00, &offset_modes_of_operation_display[2]},
    {0,2, MAXON_EPOS4_5A, 0x60FD, 0x00, &offset_digital_inputs[2]},
    {0,2, MAXON_EPOS4_5A, 0x603F, 0x00, &offset_epos4_error_code[2]},

    /* Motor 4 */
    // RxPDO
    {0,3, MAXON_EPOS4_5A, 0x6040, 0x00, &offset_control_word[3]},
    {0,3, MAXON_EPOS4_5A, 0x60FF, 0x00, &offset_target_velocity[3]},
    {0,3, MAXON_EPOS4_5A, 0x60B1, 0x00, &offset_velocity_offset[3]},
    {0,3, MAXON_EPOS4_5A, 0x6060, 0x00, &offset_modes_of_operation[3]},
    {0,3, MAXON_EPOS4_5A, 0x60FE, 0x01, &offset_digital_outputs[3]},
    {0,3, MAXON_EPOS4_5A, 0x607D, 0x01, &offset_min_position_limit[3]},
    {0,3, MAXON_EPOS4_5A, 0x607D, 0x02, &offset_max_position_limit[3]},
    // TxPDO
    {0,3, MAXON_EPOS4_5A, 0x6041, 0x00, &offset_status_word[3]},
    {0,3, MAXON_EPOS4_5A, 0x6064, 0x00, &offset_position_actual_value[3]},
    {0,3, MAXON_EPOS4_5A, 0x606C, 0x00, &offset_velocity_actual_value[3]},
    {0,3, MAXON_EPOS4_5A, 0x6077, 0x00, &offset_torque_actual_value[3]},
    {0,3, MAXON_EPOS4_5A, 0x6061, 0x00, &offset_modes_of_operation_display[3]},
    {0,3, MAXON_EPOS4_5A, 0x60FD, 0x00, &offset_digital_inputs[3]},
    {0,3, MAXON_EPOS4_5A, 0x603F, 0x00, &offset_epos4_error_code[3]},

    /* Motor 5 */
    // RxPDO
    {0,4, MAXON_EPOS4_5A, 0x6040, 0x00, &offset_control_word[4]},
    {0,4, MAXON_EPOS4_5A, 0x60FF, 0x00, &offset_target_velocity[4]},
    {0,4, MAXON_EPOS4_5A, 0x60B1, 0x00, &offset_velocity_offset[4]},
    {0,4, MAXON_EPOS4_5A, 0x6060, 0x00, &offset_modes_of_operation[4]},
    {0,4, MAXON_EPOS4_5A, 0x60FE, 0x01, &offset_digital_outputs[4]},
    {0,4, MAXON_EPOS4_5A, 0x607D, 0x01, &offset_min_position_limit[4]},
    {0,4, MAXON_EPOS4_5A, 0x607D, 0x02, &offset_max_position_limit[4]},
    // TxPDO
    {0,4, MAXON_EPOS4_5A, 0x6041, 0x00, &offset_status_word[4]},
    {0,4, MAXON_EPOS4_5A, 0x6064, 0x00, &offset_position_actual_value[4]},
    {0,4, MAXON_EPOS4_5A, 0x606C, 0x00, &offset_velocity_actual_value[4]},
    {0,4, MAXON_EPOS4_5A, 0x6077, 0x00, &offset_torque_actual_value[4]},
    {0,4, MAXON_EPOS4_5A, 0x6061, 0x00, &offset_modes_of_operation_display[4]},
    {0,4, MAXON_EPOS4_5A, 0x60FD, 0x00, &offset_digital_inputs[4]},
    {0,4, MAXON_EPOS4_5A, 0x603F, 0x00, &offset_epos4_error_code[4]},

    /* Motor 6 */
    // RxPDO
    {0,5, MAXON_EPOS4_5A, 0x6040, 0x00, &offset_control_word[5]},
    {0,5, MAXON_EPOS4_5A, 0x60FF, 0x00, &offset_target_velocity[5]},
    {0,5, MAXON_EPOS4_5A, 0x60B1, 0x00, &offset_velocity_offset[5]},
    {0,5, MAXON_EPOS4_5A, 0x6060, 0x00, &offset_modes_of_operation[5]},
    {0,5, MAXON_EPOS4_5A, 0x60FE, 0x01, &offset_digital_outputs[5]},
    {0,5, MAXON_EPOS4_5A, 0x607D, 0x01, &offset_min_position_limit[5]},
    {0,5, MAXON_EPOS4_5A, 0x607D, 0x02, &offset_max_position_limit[5]},
    // TxPDO
    {0,5, MAXON_EPOS4_5A, 0x6041, 0x00, &offset_status_word[5]},
    {0,5, MAXON_EPOS4_5A, 0x6064, 0x00, &offset_position_actual_value[5]},
    {0,5, MAXON_EPOS4_5A, 0x606C, 0x00, &offset_velocity_actual_value[5]},
    {0,5, MAXON_EPOS4_5A, 0x6077, 0x00, &offset_torque_actual_value[5]},
    {0,5, MAXON_EPOS4_5A, 0x6061, 0x00, &offset_modes_of_operation_display[5]},
    {0,5, MAXON_EPOS4_5A, 0x60FD, 0x00, &offset_digital_inputs[5]},
    {0,5, MAXON_EPOS4_5A, 0x603F, 0x00, &offset_epos4_error_code[5]},

    /* Motor 7 */
    // RxPDO
    {0,6, MAXON_EPOS4_5A, 0x6040, 0x00, &offset_control_word[6]},
    {0,6, MAXON_EPOS4_5A, 0x60FF, 0x00, &offset_target_velocity[6]},
    {0,6, MAXON_EPOS4_5A, 0x60B1, 0x00, &offset_velocity_offset[6]},
    {0,6, MAXON_EPOS4_5A, 0x6060, 0x00, &offset_modes_of_operation[6]},
    {0,6, MAXON_EPOS4_5A, 0x60FE, 0x01, &offset_digital_outputs[6]},
    {0,6, MAXON_EPOS4_5A, 0x607D, 0x01, &offset_min_position_limit[6]},
    {0,6, MAXON_EPOS4_5A, 0x607D, 0x02, &offset_max_position_limit[6]},
    // TxPDO
    {0,6, MAXON_EPOS4_5A, 0x6041, 0x00, &offset_status_word[6]},
    {0,6, MAXON_EPOS4_5A, 0x6064, 0x00, &offset_position_actual_value[6]},
    {0,6, MAXON_EPOS4_5A, 0x606C, 0x00, &offset_velocity_actual_value[6]},
    {0,6, MAXON_EPOS4_5A, 0x6077, 0x00, &offset_torque_actual_value[6]},
    {0,6, MAXON_EPOS4_5A, 0x6061, 0x00, &offset_modes_of_operation_display[6]},
    {0,6, MAXON_EPOS4_5A, 0x60FD, 0x00, &offset_digital_inputs[6]},
    {0,6, MAXON_EPOS4_5A, 0x603F, 0x00, &offset_epos4_error_code[6]},

    /* Motor 8 */
    // RxPDO
    {0,7, MAXON_EPOS4_5A, 0x6040, 0x00, &offset_control_word[7]},
    {0,7, MAXON_EPOS4_5A, 0x60FF, 0x00, &offset_target_velocity[7]},
    {0,7, MAXON_EPOS4_5A, 0x60B1, 0x00, &offset_velocity_offset[7]},
    {0,7, MAXON_EPOS4_5A, 0x6060, 0x00, &offset_modes_of_operation[7]},
    {0,7, MAXON_EPOS4_5A, 0x60FE, 0x01, &offset_digital_outputs[7]},
    {0,7, MAXON_EPOS4_5A, 0x607D, 0x01, &offset_min_position_limit[7]},
    {0,7, MAXON_EPOS4_5A, 0x607D, 0x02, &offset_max_position_limit[7]},
    // TxPDO
    {0,7, MAXON_EPOS4_5A, 0x6041, 0x00, &offset_status_word[7]},
    {0,7, MAXON_EPOS4_5A, 0x6064, 0x00, &offset_position_actual_value[7]},
    {0,7, MAXON_EPOS4_5A, 0x606C, 0x00, &offset_velocity_actual_value[7]},
    {0,7, MAXON_EPOS4_5A, 0x6077, 0x00, &offset_torque_actual_value[7]},
    {0,7, MAXON_EPOS4_5A, 0x6061, 0x00, &offset_modes_of_operation_display[7]},
    {0,7, MAXON_EPOS4_5A, 0x60FD, 0x00, &offset_digital_inputs[7]},
    {0,7, MAXON_EPOS4_5A, 0x603F, 0x00, &offset_epos4_error_code[7]}
};


/**************************** MDP module CSV mapping ****************************/
static ec_pdo_entry_info_t csv_pdo_entries[] = {
    // RxPDO (Master -> Slave)
    {0x6040, 0x00, 16},    // control word
    {0x60FF, 0x00, 32},    // target velocity
    {0x60B1, 0x00, 32},    // velocity offset
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

static ec_pdo_info_t csv_pdos[] = {
    // RxPDO(Master -> Slave) 1 mapping
    {0x1600, 7,	csv_pdo_entries + 0}, // 7개의 RxPDO entry를 mapping 할 것인데, entry의 시작 위치는 maxon_epos4_pdo_entries[0]이다. 

    // TxPDO(Master <- Slave) 1 mapping
    {0x1a00, 7,	csv_pdo_entries + 7} // 7개의 TxPDO entry를 mapping 할 것인데, entry의 시작 위치는 maxon_epos4_pdo_entries[7]이다. 
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
// define motor function: logging, cyclic task ..
// motor objects (Joint 1: Motor 1, Joint 2: Motor 2, .. Motor 7, 8 as a gripper)
EPOS4Slave motor[MOTORNUM] = {
    EPOS4Slave(1024.0f, 30.0f), EPOS4Slave(1024.0f, 8.0f), EPOS4Slave(1024.0f, 1.0f), EPOS4Slave(1024.0f, 1.0f), 
    EPOS4Slave(512.0f, 100.0f), EPOS4Slave(512.0f, 100.0f), EPOS4Slave(512.0f, 100.0f), EPOS4Slave(512.0f, 100.0f)
};

// trajectories
float trajectories[MOTORNUM][TARGET_NUM][8] = {
    // trajectory for Motor 1
    { 
        {0.0, 20.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0},
        {20.0, 20.0, 0.0, 0.0, 0.0, 0.0, 1.0, 2.0},
        {20.0, 40.0, 0.0, 0.0, 0.0, 0.0, 2.0, 4.0},
        {40.0, 40.0, 0.0, 0.0, 0.0, 0.0, 4.0, 4.5},
        {40.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.5, 6.5}
    },
    // trajectory for Motor 2
    { 
        {0.0, 20.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0},
        {20.0, 20.0, 0.0, 0.0, 0.0, 0.0, 1.0, 2.0},
        {20.0, 40.0, 0.0, 0.0, 0.0, 0.0, 2.0, 4.0},
        {40.0, 40.0, 0.0, 0.0, 0.0, 0.0, 4.0, 4.5},
        {40.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.5, 6.5}
    },
    // trajectory for Motor 3
    { 
        {0.0, 20.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0},
        {20.0, 20.0, 0.0, 0.0, 0.0, 0.0, 1.0, 2.0},
        {20.0, 40.0, 0.0, 0.0, 0.0, 0.0, 2.0, 4.0},
        {40.0, 40.0, 0.0, 0.0, 0.0, 0.0, 4.0, 4.5},
        {40.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.5, 6.5}
    },
    // trajectory for Motor 4
    { 
        {0.0, 20.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0},
        {20.0, 20.0, 0.0, 0.0, 0.0, 0.0, 1.0, 2.0},
        {20.0, 40.0, 0.0, 0.0, 0.0, 0.0, 2.0, 4.0},
        {40.0, 40.0, 0.0, 0.0, 0.0, 0.0, 4.0, 4.5},
        {40.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.5, 6.5}
    },
    // trajectory for Motor 5
    { 
        {0.0, 20.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0},
        {20.0, 20.0, 0.0, 0.0, 0.0, 0.0, 1.0, 2.0},
        {20.0, 40.0, 0.0, 0.0, 0.0, 0.0, 2.0, 4.0},
        {40.0, 40.0, 0.0, 0.0, 0.0, 0.0, 4.0, 4.5},
        {40.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.5, 6.5}
    },
    // trajectory for Motor 6
    { 
        {0.0, 20.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0},
        {20.0, 20.0, 0.0, 0.0, 0.0, 0.0, 1.0, 2.0},
        {20.0, 40.0, 0.0, 0.0, 0.0, 0.0, 2.0, 4.0},
        {40.0, 40.0, 0.0, 0.0, 0.0, 0.0, 4.0, 4.5},
        {40.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.5, 6.5}
    },
    // trajectory for Motor 7
    { 
        {0.0, 20.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0},
        {20.0, 20.0, 0.0, 0.0, 0.0, 0.0, 1.0, 2.0},
        {20.0, 40.0, 0.0, 0.0, 0.0, 0.0, 2.0, 4.0},
        {40.0, 40.0, 0.0, 0.0, 0.0, 0.0, 4.0, 4.5},
        {40.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.5, 6.5}
    },
    // trajectory for Motor 8
    { 
        {0.0, 20.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0},
        {20.0, 20.0, 0.0, 0.0, 0.0, 0.0, 1.0, 2.0},
        {20.0, 40.0, 0.0, 0.0, 0.0, 0.0, 2.0, 4.0},
        {40.0, 40.0, 0.0, 0.0, 0.0, 0.0, 4.0, 4.5},
        {40.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.5, 6.5}
    }
};
int targetIdx[MOTORNUM] = {0, };

// logging vars.
float t = 0;
int idx = 0;
float command_time[MOTORNUM] = {0.0, };

// cyclic task vars.
uint16_t status_word[MOTORNUM] = {0, };
uint16_t control_word[MOTORNUM] = {0, };
uint16_t epos4_error_code[MOTORNUM] = {0, };
uint32_t target_velocity[MOTORNUM] = {0, };
bool is_init[MOTORNUM] = {0, };
bool is_operational[MOTORNUM] = {0, };
bool is_stop[MOTORNUM] = {0, };
bool is_shutdown[MOTORNUM] = {0, };
bool is_terminate[MOTORNUM] = {0, };
bool is_new_command[MOTORNUM] = {1, 1, 1, 1, 1, 1, 1, 1};

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
        for(int i=0; i<MOTORNUM; i++){
            status_word[i] = EC_READ_U16(domain1_pd + offset_status_word[i]);
        }  
        
        // write process data on Motor 1
        switch(status_word[0] & 0x006F){   // check 0,1,2,3,5,6th bit only
            case 0x0040: // switch on disabled
                if(!is_init[0]){
                    EC_WRITE_U16(domain1_pd + offset_control_word[0], 0x0006);
                    printf("Motor 1: switch on disabled -> ready to switch on\n");
                }   
                break;

            case 0x0021: // ready to switch on
                if(!is_init[0]){
                    EC_WRITE_U16(domain1_pd + offset_control_word[0], 0x0007);
                    printf("Motor 1: ready to switch on -> switched on\n");
                }
                break;

            case 0x0023: // switched on
                if(!is_init[0]){
                    EC_WRITE_U16(domain1_pd + offset_control_word[0], 0x000F);
                    printf("Motor 1: switched on -> operation enabled\n");
                }
                break;

            case 0x0027: // operation enabled
                if(!is_init[0]){                    
                    EC_WRITE_U16(domain1_pd + offset_modes_of_operation[0], 9); // select csv mode
                    EC_WRITE_U32(domain1_pd + offset_min_position_limit[0], -1000000000); // set min pos limit
                    EC_WRITE_U32(domain1_pd + offset_max_position_limit[0], 1000000000); // set max pos limit
                    
                    is_operational[0] = 1;
                    is_init[0] = 1;                   
                } 
                
                break;

            case 0x0008: //fault
                // read error code
                epos4_error_code[0] = EC_READ_U16(domain1_pd + offset_epos4_error_code[0]);
                printf("Motor 1: Fault, error code is: 0x%04X\n", epos4_error_code[0]);

                // get controlword
                control_word[0] = EC_READ_U16(domain1_pd + offset_control_word[0]);

                // fault reset
                control_word[0] |= 0x0080; 
                EC_WRITE_U16(domain1_pd + offset_control_word[0], control_word[0]);
                is_init[0] = 0;
                break;
        }

        // write process data on Motor 2
        switch(status_word[1] & 0x006F){   // check 0,1,2,3,5,6th bit only
            case 0x0040: // switch on disabled
                if(!is_init[1]){ 
                    EC_WRITE_U16(domain1_pd + offset_control_word[1], 0x0006);
                    printf("Motor 2: switch on disabled -> ready to switch on\n");
                }
                break;

            case 0x0021: // ready to switch on
                if(!is_init[1]){ 
                    EC_WRITE_U16(domain1_pd + offset_control_word[1], 0x0007);
                    printf("Motor 2: ready to switch on -> switched on\n");
                }
                break;

            case 0x0023: // switched on
                if(!is_init[1]){ 
                    EC_WRITE_U16(domain1_pd + offset_control_word[1], 0x000F);
                    printf("Motor 2: switched on -> operation enabled\n");
                }
                break;

            case 0x0027: // operation enabled
                if(!is_init[1]){                    
                    EC_WRITE_U16(domain1_pd + offset_modes_of_operation[1], 9); // select csv mode
                    EC_WRITE_U32(domain1_pd + offset_min_position_limit[1], -1000000000); // set min pos limit
                    EC_WRITE_U32(domain1_pd + offset_max_position_limit[1], 1000000000); // set max pos limit
                    
                    is_operational[1] = 1;
                    is_init[1] = 1;                   
                } 
                
                break;

            case 0x0008: //fault
                // read error code
                epos4_error_code[1] = EC_READ_U16(domain1_pd + offset_epos4_error_code[1]);
                printf("Motor 2: Fault, error code is: 0x%04X\n", epos4_error_code[1]);

                // get controlword
                control_word[1] = EC_READ_U16(domain1_pd + offset_control_word[1]);

                // fault reset
                control_word[1] |= 0x0080; 
                EC_WRITE_U16(domain1_pd + offset_control_word[1], control_word[1]);
                is_init[1] = 0;  
                break;
        }

        // write process data on Motor 3
        switch(status_word[2] & 0x006F){   // check 0,1,2,3,5,6th bit only
            case 0x0040: // switch on disabled
                if(!is_init[2]){ 
                    EC_WRITE_U16(domain1_pd + offset_control_word[2], 0x0006);
                    printf("Motor 3: switch on disabled -> ready to switch on\n");
                }
                break;

            case 0x0021: // ready to switch on
                if(!is_init[2]){ 
                    EC_WRITE_U16(domain1_pd + offset_control_word[2], 0x0007);
                    printf("Motor 3: ready to switch on -> switched on\n");
                }
                break;

            case 0x0023: // switched on
                if(!is_init[2]){ 
                    EC_WRITE_U16(domain1_pd + offset_control_word[2], 0x000F);
                    printf("Motor 3: switched on -> operation enabled\n");
                }
                break;

            case 0x0027: // operation enabled
                if(!is_init[2]){                    
                    EC_WRITE_U16(domain1_pd + offset_modes_of_operation[2], 9); // select csv mode
                    EC_WRITE_U32(domain1_pd + offset_min_position_limit[2], -1000000000); // set min pos limit
                    EC_WRITE_U32(domain1_pd + offset_max_position_limit[2], 1000000000); // set max pos limit
                    
                    is_operational[2] = 1;
                    is_init[2] = 1;                   
                } 
                
                break;

            case 0x0008: //fault
                // read error code
                epos4_error_code[2] = EC_READ_U16(domain1_pd + offset_epos4_error_code[2]);
                printf("Motor 3: Fault, error code is: 0x%04X\n", epos4_error_code[2]);

                // get controlword
                control_word[2] = EC_READ_U16(domain1_pd + offset_control_word[2]);

                // fault reset
                control_word[2] |= 0x0080; 
                EC_WRITE_U16(domain1_pd + offset_control_word[2], control_word[2]);
                is_init[2] = 0;  
                break;
        }

        // write process data on Motor 4
        switch(status_word[3] & 0x006F){   // check 0,1,2,3,5,6th bit only
            case 0x0040: // switch on disabled
                if(!is_init[3]){ 
                    EC_WRITE_U16(domain1_pd + offset_control_word[3], 0x0006);
                    printf("Motor 4: switch on disabled -> ready to switch on\n");
                }
                break;

            case 0x0021: // ready to switch on
                if(!is_init[3]){ 
                    EC_WRITE_U16(domain1_pd + offset_control_word[3], 0x0007);
                    printf("Motor 4: ready to switch on -> switched on\n");
                }
                break;

            case 0x0023: // switched on
                if(!is_init[3]){ 
                    EC_WRITE_U16(domain1_pd + offset_control_word[3], 0x000F);
                    printf("Motor 4: switched on -> operation enabled\n");
                }
                break;

            case 0x0027: // operation enabled
                if(!is_init[3]){                    
                    EC_WRITE_U16(domain1_pd + offset_modes_of_operation[3], 9); // select csv mode
                    EC_WRITE_U32(domain1_pd + offset_min_position_limit[3], -1000000000); // set min pos limit
                    EC_WRITE_U32(domain1_pd + offset_max_position_limit[3], 1000000000); // set max pos limit
                    
                    is_operational[3] = 1;
                    is_init[3] = 1;                   
                } 
                
                break;

            case 0x0008: //fault
                // read error code
                epos4_error_code[3] = EC_READ_U16(domain1_pd + offset_epos4_error_code[3]);
                printf("Motor 4: Fault, error code is: 0x%04X\n", epos4_error_code[3]);

                // get controlword
                control_word[3] = EC_READ_U16(domain1_pd + offset_control_word[3]);

                // fault reset
                control_word[3] |= 0x0080; 
                EC_WRITE_U16(domain1_pd + offset_control_word[3], control_word[3]);
                is_init[3] = 0;  
                break;
        }

        // write process data on Motor 5
        switch(status_word[4] & 0x006F){   // check 0,1,2,3,5,6th bit only
            case 0x0040: // switch on disabled
                if(!is_init[4]){ 
                    EC_WRITE_U16(domain1_pd + offset_control_word[4], 0x0006);
                    printf("Motor 5: switch on disabled -> ready to switch on\n");
                }
                break;

            case 0x0021: // ready to switch on
                if(!is_init[4]){ 
                    EC_WRITE_U16(domain1_pd + offset_control_word[4], 0x0007);
                    printf("Motor 5: ready to switch on -> switched on\n");
                }
                break;

            case 0x0023: // switched on
                if(!is_init[4]){ 
                    EC_WRITE_U16(domain1_pd + offset_control_word[4], 0x000F);
                    printf("Motor 5: switched on -> operation enabled\n");
                }
                break;

            case 0x0027: // operation enabled
                if(!is_init[4]){                    
                    EC_WRITE_U16(domain1_pd + offset_modes_of_operation[4], 9); // select csv mode
                    EC_WRITE_U32(domain1_pd + offset_min_position_limit[4], -1000000000); // set min pos limit
                    EC_WRITE_U32(domain1_pd + offset_max_position_limit[4], 1000000000); // set max pos limit
                    
                    is_operational[4] = 1;
                    is_init[4] = 1;                   
                } 
                
                break;

            case 0x0008: //fault
                // read error code
                epos4_error_code[4] = EC_READ_U16(domain1_pd + offset_epos4_error_code[4]);
                printf("Motor 5: Fault, error code is: 0x%04X\n", epos4_error_code[4]);

                // get controlword
                control_word[4] = EC_READ_U16(domain1_pd + offset_control_word[4]);

                // fault reset
                control_word[4] |= 0x0080; 
                EC_WRITE_U16(domain1_pd + offset_control_word[4], control_word[4]);
                is_init[4] = 0;  
                break;
        }

        // write process data on Motor 6
        switch(status_word[5] & 0x006F){   // check 0,1,2,3,5,6th bit only
            case 0x0040: // switch on disabled
                if(!is_init[5]){ 
                    EC_WRITE_U16(domain1_pd + offset_control_word[5], 0x0006);
                    printf("Motor 6: switch on disabled -> ready to switch on\n");
                }
                break;

            case 0x0021: // ready to switch on
                if(!is_init[5]){ 
                    EC_WRITE_U16(domain1_pd + offset_control_word[5], 0x0007);
                    printf("Motor 6: ready to switch on -> switched on\n");
                }
                break;

            case 0x0023: // switched on
                if(!is_init[5]){ 
                    EC_WRITE_U16(domain1_pd + offset_control_word[5], 0x000F);
                    printf("Motor 6: switched on -> operation enabled\n");
                }
                break;

            case 0x0027: // operation enabled
                if(!is_init[5]){                    
                    EC_WRITE_U16(domain1_pd + offset_modes_of_operation[5], 9); // select csv mode
                    EC_WRITE_U32(domain1_pd + offset_min_position_limit[5], -1000000000); // set min pos limit
                    EC_WRITE_U32(domain1_pd + offset_max_position_limit[5], 1000000000); // set max pos limit
                    
                    is_operational[5] = 1;
                    is_init[5] = 1;                   
                } 
                
                break;

            case 0x0008: //fault
                // read error code
                epos4_error_code[5] = EC_READ_U16(domain1_pd + offset_epos4_error_code[5]);
                printf("Motor 6: Fault, error code is: 0x%04X\n", epos4_error_code[5]);

                // get controlword
                control_word[5] = EC_READ_U16(domain1_pd + offset_control_word[5]);

                // fault reset
                control_word[5] |= 0x0080; 
                EC_WRITE_U16(domain1_pd + offset_control_word[5], control_word[5]);
                is_init[5] = 0;  
                break;
        }

        // write process data on Motor 7
        switch(status_word[6] & 0x006F){   // check 0,1,2,3,5,6th bit only
            case 0x0040: // switch on disabled
                if(!is_init[6]){ 
                    EC_WRITE_U16(domain1_pd + offset_control_word[6], 0x0006);
                    printf("Motor 7: switch on disabled -> ready to switch on\n");
                }
                break;

            case 0x0021: // ready to switch on
                if(!is_init[6]){ 
                    EC_WRITE_U16(domain1_pd + offset_control_word[6], 0x0007);
                    printf("Motor 7: ready to switch on -> switched on\n");
                }
                break;

            case 0x0023: // switched on
                if(!is_init[6]){ 
                    EC_WRITE_U16(domain1_pd + offset_control_word[6], 0x000F);
                    printf("Motor 7: switched on -> operation enabled\n");
                }
                break;

            case 0x0027: // operation enabled
                if(!is_init[6]){                    
                    EC_WRITE_U16(domain1_pd + offset_modes_of_operation[6], 9); // select csv mode
                    EC_WRITE_U32(domain1_pd + offset_min_position_limit[6], -1000000000); // set min pos limit
                    EC_WRITE_U32(domain1_pd + offset_max_position_limit[6], 1000000000); // set max pos limit
                    
                    is_operational[6] = 1;
                    is_init[6] = 1;                   
                } 
                
                break;

            case 0x0008: //fault
                // read error code
                epos4_error_code[6] = EC_READ_U16(domain1_pd + offset_epos4_error_code[6]);
                printf("Motor 7: Fault, error code is: 0x%04X\n", epos4_error_code[6]);

                // get controlword
                control_word[6] = EC_READ_U16(domain1_pd + offset_control_word[6]);

                // fault reset
                control_word[6] |= 0x0080; 
                EC_WRITE_U16(domain1_pd + offset_control_word[6], control_word[6]);
                is_init[6] = 0;  
                break;
        }

        // write process data on Motor 8
        switch(status_word[7] & 0x006F){   // check 0,1,2,3,5,6th bit only
            case 0x0040: // switch on disabled
                if(!is_init[7]){ 
                    EC_WRITE_U16(domain1_pd + offset_control_word[7], 0x0006);
                    printf("Motor 8: switch on disabled -> ready to switch on\n");
                }
                break;

            case 0x0021: // ready to switch on
                if(!is_init[7]){ 
                    EC_WRITE_U16(domain1_pd + offset_control_word[7], 0x0007);
                    printf("Motor 8: ready to switch on -> switched on\n");
                }
                break;

            case 0x0023: // switched on
                if(!is_init[7]){ 
                    EC_WRITE_U16(domain1_pd + offset_control_word[7], 0x000F);
                    printf("Motor 8: switched on -> operation enabled\n");
                }
                break;

            case 0x0027: // operation enabled
                if(!is_init[7]){                    
                    EC_WRITE_U16(domain1_pd + offset_modes_of_operation[7], 9); // select csv mode
                    EC_WRITE_U32(domain1_pd + offset_min_position_limit[7], -1000000000); // set min pos limit
                    EC_WRITE_U32(domain1_pd + offset_max_position_limit[7], 1000000000); // set max pos limit
                    
                    is_operational[7] = 1;
                    is_init[7] = 1;                   
                } 
                
                break;

            case 0x0008: //fault
                // read error code
                epos4_error_code[7] = EC_READ_U16(domain1_pd + offset_epos4_error_code[7]);
                printf("Motor 8: Fault, error code is: 0x%04X\n", epos4_error_code[7]);

                // get controlword
                control_word[7] = EC_READ_U16(domain1_pd + offset_control_word[7]);

                // fault reset
                control_word[7] |= 0x0080; 
                EC_WRITE_U16(domain1_pd + offset_control_word[7], control_word[7]);
                is_init[7] = 0;  
                break;
        }
    } 

    // set new trajectory
    if(is_new_command[0]){
        motor[0].setTrajectoryParam(trajectories[0][targetIdx[0]][0], trajectories[0][targetIdx[0]][1], trajectories[0][targetIdx[0]][2], trajectories[0][targetIdx[0]][3], trajectories[0][targetIdx[0]][4], trajectories[0][targetIdx[0]][5], trajectories[0][targetIdx[0]][6], trajectories[0][targetIdx[0]][7]);
        command_time[0] = command_time[0] + motor[0].getMoveTime();
        is_new_command[0] = 0;
        cout << endl << "Motor 1: Trajectory parameters initialized." << endl;
    }
    if(is_new_command[1]){
        motor[1].setTrajectoryParam(trajectories[1][targetIdx[1]][0], trajectories[1][targetIdx[1]][1], trajectories[1][targetIdx[1]][2], trajectories[1][targetIdx[1]][3], trajectories[1][targetIdx[1]][4], trajectories[1][targetIdx[1]][5], trajectories[1][targetIdx[1]][6], trajectories[1][targetIdx[1]][7]);
        command_time[1] = command_time[1] + motor[1].getMoveTime();
        is_new_command[1] = 0;
        cout << endl << "Motor 2: Trajectory parameters initialized." << endl;
    }
     if(is_new_command[2]){
        motor[2].setTrajectoryParam(trajectories[2][targetIdx[2]][0], trajectories[2][targetIdx[2]][1], trajectories[2][targetIdx[2]][2], trajectories[2][targetIdx[2]][3], trajectories[2][targetIdx[2]][4], trajectories[2][targetIdx[2]][5], trajectories[2][targetIdx[2]][6], trajectories[2][targetIdx[2]][7]);
        command_time[2] = command_time[2] + motor[2].getMoveTime();
        is_new_command[2] = 0;
        cout << endl << "Motor 3: Trajectory parameters initialized." << endl;
    }
     if(is_new_command[3]){
        motor[3].setTrajectoryParam(trajectories[3][targetIdx[3]][0], trajectories[3][targetIdx[3]][1], trajectories[3][targetIdx[3]][2], trajectories[3][targetIdx[3]][3], trajectories[3][targetIdx[3]][4], trajectories[3][targetIdx[3]][5], trajectories[3][targetIdx[3]][6], trajectories[3][targetIdx[3]][7]);
        command_time[3] = command_time[3] + motor[3].getMoveTime();
        is_new_command[3] = 0;
        cout << endl << "Motor 4: Trajectory parameters initialized." << endl;
    }
     if(is_new_command[4]){
        motor[4].setTrajectoryParam(trajectories[4][targetIdx[4]][0], trajectories[4][targetIdx[4]][1], trajectories[4][targetIdx[4]][2], trajectories[4][targetIdx[4]][3], trajectories[4][targetIdx[4]][4], trajectories[4][targetIdx[4]][5], trajectories[4][targetIdx[4]][6], trajectories[4][targetIdx[4]][7]);
        command_time[4] = command_time[4] + motor[4].getMoveTime();
        is_new_command[4] = 0;
        cout << endl << "Motor 5: Trajectory parameters initialized." << endl;
    }
     if(is_new_command[5]){
        motor[5].setTrajectoryParam(trajectories[5][targetIdx[5]][0], trajectories[5][targetIdx[5]][1], trajectories[5][targetIdx[5]][2], trajectories[5][targetIdx[5]][3], trajectories[5][targetIdx[5]][4], trajectories[5][targetIdx[5]][5], trajectories[5][targetIdx[5]][6], trajectories[5][targetIdx[5]][7]);
        command_time[5] = command_time[5] + motor[5].getMoveTime();
        is_new_command[5] = 0;
        cout << endl << "Motor 6: Trajectory parameters initialized." << endl;
    }
     if(is_new_command[6]){
        motor[6].setTrajectoryParam(trajectories[6][targetIdx[6]][0], trajectories[6][targetIdx[6]][1], trajectories[6][targetIdx[6]][2], trajectories[6][targetIdx[6]][3], trajectories[6][targetIdx[6]][4], trajectories[6][targetIdx[6]][5], trajectories[6][targetIdx[6]][6], trajectories[6][targetIdx[6]][7]);
        command_time[6] = command_time[6] + motor[6].getMoveTime();
        is_new_command[6] = 0;
        cout << endl << "Motor 7: Trajectory parameters initialized." << endl;
    }
     if(is_new_command[7]){
        motor[7].setTrajectoryParam(trajectories[7][targetIdx[7]][0], trajectories[7][targetIdx[7]][1], trajectories[7][targetIdx[7]][2], trajectories[7][targetIdx[7]][3], trajectories[7][targetIdx[7]][4], trajectories[7][targetIdx[7]][5], trajectories[7][targetIdx[7]][6], trajectories[7][targetIdx[7]][7]);
        command_time[7] = command_time[7] + motor[7].getMoveTime();
        is_new_command[7] = 0;
        cout << endl << "Motor 8: Trajectory parameters initialized." << endl;
    }

    // apply trajectory and do logging
    if(is_operational[0] * is_operational[1] * is_operational[2] * is_operational[3] * is_operational[4] * is_operational[5] * is_operational[6] * is_operational[7]){
        // set target velocity by following 5th-poly trajectory
        motor[0].setTrajectory(t);
        motor[1].setTrajectory(t);
        motor[2].setTrajectory(t);
        motor[3].setTrajectory(t);
        motor[4].setTrajectory(t);
        motor[5].setTrajectory(t);
        motor[6].setTrajectory(t);
        motor[7].setTrajectory(t);

        // write a target velocity
        EC_WRITE_U32(domain1_pd + offset_target_velocity[0], motor[0].getVelTick());
        EC_WRITE_U32(domain1_pd + offset_target_velocity[1], motor[1].getVelTick());
        EC_WRITE_U32(domain1_pd + offset_target_velocity[2], motor[2].getVelTick());
        EC_WRITE_U32(domain1_pd + offset_target_velocity[3], motor[3].getVelTick());
        EC_WRITE_U32(domain1_pd + offset_target_velocity[4], motor[4].getVelTick());
        EC_WRITE_U32(domain1_pd + offset_target_velocity[5], motor[5].getVelTick());
        EC_WRITE_U32(domain1_pd + offset_target_velocity[6], motor[6].getVelTick());
        EC_WRITE_U32(domain1_pd + offset_target_velocity[7], motor[7].getVelTick());

        // logging
        motor[0].logging(t, (float)EC_READ_S32(domain1_pd + offset_velocity_actual_value[0]), (float)EC_READ_S32(domain1_pd + offset_position_actual_value[0]));
        motor[1].logging(t, (float)EC_READ_S32(domain1_pd + offset_velocity_actual_value[1]), (float)EC_READ_S32(domain1_pd + offset_position_actual_value[1]));
        motor[2].logging(t, (float)EC_READ_S32(domain1_pd + offset_velocity_actual_value[2]), (float)EC_READ_S32(domain1_pd + offset_position_actual_value[2]));
        motor[3].logging(t, (float)EC_READ_S32(domain1_pd + offset_velocity_actual_value[3]), (float)EC_READ_S32(domain1_pd + offset_position_actual_value[3]));
        motor[4].logging(t, (float)EC_READ_S32(domain1_pd + offset_velocity_actual_value[4]), (float)EC_READ_S32(domain1_pd + offset_position_actual_value[4]));
        motor[5].logging(t, (float)EC_READ_S32(domain1_pd + offset_velocity_actual_value[5]), (float)EC_READ_S32(domain1_pd + offset_position_actual_value[5]));
        motor[6].logging(t, (float)EC_READ_S32(domain1_pd + offset_velocity_actual_value[6]), (float)EC_READ_S32(domain1_pd + offset_position_actual_value[6]));
        motor[7].logging(t, (float)EC_READ_S32(domain1_pd + offset_velocity_actual_value[7]), (float)EC_READ_S32(domain1_pd + offset_position_actual_value[7]));
        t += 0.001; 
        idx++;
    } 

    // stop the motor
    if(is_stop[0]){        
        EC_WRITE_U32(domain1_pd + offset_target_velocity[0], 0);
        if(EC_READ_S32(domain1_pd + offset_velocity_actual_value[0]) == 0){
            printf("Motor 1 is stopped.\n");
            is_operational[0] = 0;
            is_stop[0] = 0;
            is_shutdown[0] = 1;
        } 
    }
    if(is_stop[1]){        
        EC_WRITE_U32(domain1_pd + offset_target_velocity[1], 0);
        if(EC_READ_S32(domain1_pd + offset_velocity_actual_value[1]) == 0){
            printf("Motor 2 is stopped.\n");
            is_operational[1] = 0;
            is_stop[1] = 0;
            is_shutdown[1] = 1;
        } 
    }
    if(is_stop[2]){        
        EC_WRITE_U32(domain1_pd + offset_target_velocity[2], 0);
        if(EC_READ_S32(domain1_pd + offset_velocity_actual_value[2]) == 0){
            printf("Motor 3 is stopped.\n");
            is_operational[2] = 0;
            is_stop[2] = 0;
            is_shutdown[2] = 1;
        } 
    }
    if(is_stop[3]){        
        EC_WRITE_U32(domain1_pd + offset_target_velocity[3], 0);
        if(EC_READ_S32(domain1_pd + offset_velocity_actual_value[3]) == 0){
            printf("Motor 4 is stopped.\n");
            is_operational[3] = 0;
            is_stop[3] = 0;
            is_shutdown[3] = 1;
        } 
    }
    if(is_stop[4]){        
        EC_WRITE_U32(domain1_pd + offset_target_velocity[4], 0);
        if(EC_READ_S32(domain1_pd + offset_velocity_actual_value[4]) == 0){
            printf("Motor 5 is stopped.\n");
            is_operational[4] = 0;
            is_stop[4] = 0;
            is_shutdown[4] = 1;
        } 
    }
    if(is_stop[5]){        
        EC_WRITE_U32(domain1_pd + offset_target_velocity[5], 0);
        if(EC_READ_S32(domain1_pd + offset_velocity_actual_value[5]) == 0){
            printf("Motor 6 is stopped.\n");
            is_operational[5] = 0;
            is_stop[5] = 0;
            is_shutdown[5] = 1;
        } 
    }
    if(is_stop[6]){        
        EC_WRITE_U32(domain1_pd + offset_target_velocity[6], 0);
        if(EC_READ_S32(domain1_pd + offset_velocity_actual_value[6]) == 0){
            printf("Motor 7 is stopped.\n");
            is_operational[6] = 0;
            is_stop[6] = 0;
            is_shutdown[6] = 1;
        } 
    }
    if(is_stop[7]){        
        EC_WRITE_U32(domain1_pd + offset_target_velocity[7], 0);
        if(EC_READ_S32(domain1_pd + offset_velocity_actual_value[7]) == 0){
            printf("Motor 8 is stopped.\n");
            is_operational[7] = 0;
            is_stop[7] = 0;
            is_shutdown[7] = 1;
        } 
    }

    // init the motor controller: switch on disabled
    if(is_shutdown[0] * is_shutdown[1] * is_shutdown[2] * is_shutdown[3] * is_shutdown[4] * is_shutdown[5] * is_shutdown[6] * is_shutdown[7]){
        EC_WRITE_U16(domain1_pd + offset_control_word[0], 0x0000);
        EC_WRITE_U16(domain1_pd + offset_control_word[1], 0x0000);
        EC_WRITE_U16(domain1_pd + offset_control_word[2], 0x0000);
        EC_WRITE_U16(domain1_pd + offset_control_word[3], 0x0000);
        EC_WRITE_U16(domain1_pd + offset_control_word[4], 0x0000);
        EC_WRITE_U16(domain1_pd + offset_control_word[5], 0x0000);
        EC_WRITE_U16(domain1_pd + offset_control_word[6], 0x0000);
        EC_WRITE_U16(domain1_pd + offset_control_word[7], 0x0000);

        if(((EC_READ_U16(domain1_pd + offset_status_word[0])) & 0x0040) == 0x0040){
            printf("Motor 1: operation enabled -> switch on disabled.\n");
            is_operational[0] = 0;
            is_shutdown[0] = 0;
            is_terminate[0] = 1;
        }
        if(((EC_READ_U16(domain1_pd + offset_status_word[1])) & 0x0040) == 0x0040){
            printf("Motor 2: operation enabled -> switch on disabled.\n");
            is_operational[1] = 0;
            is_shutdown[1] = 0;
            is_terminate[1] = 1;
        }
        if(((EC_READ_U16(domain1_pd + offset_status_word[2])) & 0x0040) == 0x0040){
            printf("Motor 3: operation enabled -> switch on disabled.\n");
            is_operational[2] = 0;
            is_shutdown[2] = 0;
            is_terminate[2] = 1;
        }
        if(((EC_READ_U16(domain1_pd + offset_status_word[3])) & 0x0040) == 0x0040){
            printf("Motor 4: operation enabled -> switch on disabled.\n");
            is_operational[3] = 0;
            is_shutdown[3] = 0;
            is_terminate[3] = 1;
        }
        if(((EC_READ_U16(domain1_pd + offset_status_word[4])) & 0x0040) == 0x0040){
            printf("Motor 5: operation enabled -> switch on disabled.\n");
            is_operational[4] = 0;
            is_shutdown[4] = 0;
            is_terminate[4] = 1;
        }
        if(((EC_READ_U16(domain1_pd + offset_status_word[5])) & 0x0040) == 0x0040){
            printf("Motor 6: operation enabled -> switch on disabled.\n");
            is_operational[5] = 0;
            is_shutdown[5] = 0;
            is_terminate[5] = 1;
        }
        if(((EC_READ_U16(domain1_pd + offset_status_word[6])) & 0x0040) == 0x0040){
            printf("Motor 7: operation enabled -> switch on disabled.\n");
            is_operational[6] = 0;
            is_shutdown[6] = 0;
            is_terminate[6] = 1;
        }
        if(((EC_READ_U16(domain1_pd + offset_status_word[7])) & 0x0040) == 0x0040){
            printf("Motor 8: operation enabled -> switch on disabled.\n");
            is_operational[7] = 0;
            is_shutdown[7] = 0;
            is_terminate[7] = 1;
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

    // slave 1
    if (!(slave_config = ecrt_master_slave_config(master, 0, 0, MAXON_EPOS4_5A))) 
    {
        fprintf(stderr, "Failed to get slave 1 configuration.\n");
        return -1;
    }
    printf("Configuring PDOs of Slave 1...\n");
    if (ecrt_slave_config_pdos(slave_config, EC_END, maxon_epos4_syncs_csv)) 
    {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }
    // slave 2
    if (!(slave_config = ecrt_master_slave_config(master, 0, 1, MAXON_EPOS4_5A))) 
    {
        fprintf(stderr, "Failed to get slave 2 configuration.\n");
        return -1;
    }
    printf("Configuring PDOs of Slave 2...\n");
    if (ecrt_slave_config_pdos(slave_config, EC_END, maxon_epos4_syncs_csv)) 
    {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }
    // slave 3
    if (!(slave_config = ecrt_master_slave_config(master, 0, 2, MAXON_EPOS4_5A))) 
    {
        fprintf(stderr, "Failed to get slave 3 configuration.\n");
        return -1;
    }
    printf("Configuring PDOs of Slave 3...\n");
    if (ecrt_slave_config_pdos(slave_config, EC_END, maxon_epos4_syncs_csv)) 
    {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }
    // slave 4
    if (!(slave_config = ecrt_master_slave_config(master, 0, 3, MAXON_EPOS4_5A))) 
    {
        fprintf(stderr, "Failed to get slave 4 configuration.\n");
        return -1;
    }
    printf("Configuring PDOs of Slave 4...\n");
    if (ecrt_slave_config_pdos(slave_config, EC_END, maxon_epos4_syncs_csv)) 
    {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }
    // slave 5
    if (!(slave_config = ecrt_master_slave_config(master, 0, 4, MAXON_EPOS4_5A))) 
    {
        fprintf(stderr, "Failed to get slave 5 configuration.\n");
        return -1;
    }
    printf("Configuring PDOs of Slave 5...\n");
    if (ecrt_slave_config_pdos(slave_config, EC_END, maxon_epos4_syncs_csv)) 
    {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }
    // slave 6
    if (!(slave_config = ecrt_master_slave_config(master, 0, 5, MAXON_EPOS4_5A))) 
    {
        fprintf(stderr, "Failed to get slave 6 configuration.\n");
        return -1;
    }
    printf("Configuring PDOs of Slave 6...\n");
    if (ecrt_slave_config_pdos(slave_config, EC_END, maxon_epos4_syncs_csv)) 
    {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }
    // slave 7
    if (!(slave_config = ecrt_master_slave_config(master, 0, 6, MAXON_EPOS4_5A))) 
    {
        fprintf(stderr, "Failed to get slave 7 configuration.\n");
        return -1;
    }
    printf("Configuring PDOs of Slave 7...\n");
    if (ecrt_slave_config_pdos(slave_config, EC_END, maxon_epos4_syncs_csv)) 
    {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }
    // slave 8
    if (!(slave_config = ecrt_master_slave_config(master, 0, 7, MAXON_EPOS4_5A))) 
    {
        fprintf(stderr, "Failed to get slave 8 configuration.\n");
        return -1;
    }
    printf("Configuring PDOs of Slave 8...\n");
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

    printf("Using priority %i.\n", param.sched_priority);
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

        if(stopSignal == 'q'){
            printf("@@ Emergency Stop @@\n");
            for(int i=0; i<MOTORNUM; i++){
                is_stop[i] = 1;
                is_operational[i] = 0;
            }            
        }

        if(is_terminate[0] == 1 || is_terminate[1] == 1 || is_terminate[2] == 1 || is_terminate[3] == 1 || is_terminate[4] == 1 || is_terminate[5] == 1 || is_terminate[6] == 1 || is_terminate[7] == 1){
            printf("Terminate the program. Bye~ \n");
            break;
        }

        // Motor 1 handling
        if((is_operational[0]) && (!is_shutdown[0]) && (targetIdx[0] < TARGET_NUM) && (idx > command_time[0]*1000)){
            targetIdx[0]++;
            if(targetIdx[0] >= TARGET_NUM){
                printf("\nMotor1: All the targets are finished\n");
                is_operational[0] = 0;
                is_stop[0] = 1;
            }
            else{
                // printf("Continue to the next target\n");
                is_new_command[0] = 1;
            } 
        }       
        // Motor 2 handling
        if((is_operational[1]) && (!is_shutdown[1]) && (targetIdx[1] < TARGET_NUM) && (idx > command_time[1]*1000)){
            targetIdx[1]++;
            if(targetIdx[1] >= TARGET_NUM){
                printf("\nMotor 2: All the targets are finished\n");
                is_operational[1] = 0;
                is_stop[1] = 1;
            }
            else{
                // printf("Continue to the next target\n");
                is_new_command[1] = 1;
            } 
        }        
        // Motor 3 handling
        if((is_operational[2]) && (!is_shutdown[2]) && (targetIdx[2] < TARGET_NUM) && (idx > command_time[2]*1000)){
            targetIdx[2]++;
            if(targetIdx[2] >= TARGET_NUM){
                printf("\nMotor 3: All the targets are finished\n");
                is_operational[2] = 0;
                is_stop[2] = 1;
            }
            else{
                // printf("Continue to the next target\n");
                is_new_command[2] = 1;
            } 
        }  
        // Motor 4 handling
        if((is_operational[3]) && (!is_shutdown[3]) && (targetIdx[3] < TARGET_NUM) && (idx > command_time[3]*1000)){
            targetIdx[3]++;
            if(targetIdx[3] >= TARGET_NUM){
                printf("\nMotor 4: All the targets are finished\n");
                is_operational[3] = 0;
                is_stop[3] = 1;
            }
            else{
                // printf("Continue to the next target\n");
                is_new_command[3] = 1;
            } 
        }  
        // Motor 5 handling
        if((is_operational[4]) && (!is_shutdown[4]) && (targetIdx[4] < TARGET_NUM) && (idx > command_time[4]*1000)){
            targetIdx[4]++;
            if(targetIdx[4] >= TARGET_NUM){
                printf("\nMotor 5: All the targets are finished\n");
                is_operational[4] = 0;
                is_stop[4] = 1;
            }
            else{
                // printf("Continue to the next target\n");
                is_new_command[4] = 1;
            } 
        }  
        // Motor 6 handling
        if((is_operational[5]) && (!is_shutdown[5]) && (targetIdx[5] < TARGET_NUM) && (idx > command_time[5]*1000)){
            targetIdx[5]++;
            if(targetIdx[5] >= TARGET_NUM){
                printf("\nMotor 6: All the targets are finished\n");
                is_operational[5] = 0;
                is_stop[5] = 1;
            }
            else{
                // printf("Continue to the next target\n");
                is_new_command[5] = 1;
            } 
        }  
        // Motor 7 handling
        if((is_operational[6]) && (!is_shutdown[6]) && (targetIdx[6] < TARGET_NUM) && (idx > command_time[6]*1000)){
            targetIdx[6]++;
            if(targetIdx[6] >= TARGET_NUM){
                printf("\nMotor 7: All the targets are finished\n");
                is_operational[6] = 0;
                is_stop[6] = 1;
            }
            else{
                // printf("Continue to the next target\n");
                is_new_command[6] = 1;
            } 
        }  
        // Motor 8 handling
        if((is_operational[7]) && (!is_shutdown[7]) && (targetIdx[7] < TARGET_NUM) && (idx > command_time[7]*1000)){
            targetIdx[7]++;
            if(targetIdx[7] >= TARGET_NUM){
                printf("\nMotor 8: All the targets are finished\n");
                is_operational[7] = 0;
                is_stop[7] = 1;
            }
            else{
                // printf("Continue to the next target\n");
                is_new_command[7] = 1;
            } 
        }  

        wakeup_time.tv_nsec += PERIOD_NS;
        while (wakeup_time.tv_nsec >= NSEC_PER_SEC) {
            wakeup_time.tv_nsec -= NSEC_PER_SEC;
            wakeup_time.tv_sec++;
        }
    }

    motor[0].saveData("/home/robogram/motor_ws/CSVMotorController/logging/robot_M1_pos01.txt", "/home/robogram/motor_ws/CSVMotorController/logging/robot_M1_vel01.txt");
    motor[1].saveData("/home/robogram/motor_ws/CSVMotorController/logging/robot_M2_pos01.txt", "/home/robogram/motor_ws/CSVMotorController/logging/robot_M2_vel01.txt");
    motor[2].saveData("/home/robogram/motor_ws/CSVMotorController/logging/robot_M3_pos01.txt", "/home/robogram/motor_ws/CSVMotorController/logging/robot_M3_vel01.txt");
    motor[3].saveData("/home/robogram/motor_ws/CSVMotorController/logging/robot_M4_pos01.txt", "/home/robogram/motor_ws/CSVMotorController/logging/robot_M4_vel01.txt");
    motor[4].saveData("/home/robogram/motor_ws/CSVMotorController/logging/robot_M5_pos01.txt", "/home/robogram/motor_ws/CSVMotorController/logging/robot_M5_vel01.txt");
    motor[5].saveData("/home/robogram/motor_ws/CSVMotorController/logging/robot_M6_pos01.txt", "/home/robogram/motor_ws/CSVMotorController/logging/robot_M6_vel01.txt");
    motor[6].saveData("/home/robogram/motor_ws/CSVMotorController/logging/robot_M7_pos01.txt", "/home/robogram/motor_ws/CSVMotorController/logging/robot_M7_vel01.txt");
    motor[7].saveData("/home/robogram/motor_ws/CSVMotorController/logging/robot_M8_pos01.txt", "/home/robogram/motor_ws/CSVMotorController/logging/robot_M8_vel01.txt");

    return ret;
}
/****************************************************************************/

