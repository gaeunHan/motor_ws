// CSPMotorController.h

#ifndef CSP_MOTOR_CONTROLLER_H
#define CSP_MOTOR_CONTROLLER_H

#include <vector>
#include <string>
#include "ecrt.h"

/****************************************************************************/
#define MAXON_EPOS4_5A 0x000000fb, 0x61500000 // Vendor ID, Product Code
#define PERIOD_NS   (1000000) // Task period in ns
#define NSEC_PER_SEC (1000000000) // The maximum stack size which is guranteed safe to access without faulting
#define FREQUENCY (NSEC_PER_SEC / PERIOD_NS)
#define PULSE (1024.0 * 4.0)
#define GEAR_RATIO 35.0
#define CNT_PER_DEGREE (PULSE * GEAR_RATIO) / 360.0 // 398.0
/****************************************************************************/

class CSPMotorController {
private:
    /* EtherCAT */
    static ec_master_t *master;
    static ec_master_state_t master_state;
    static ec_domain_t *domain1;
    static ec_domain_state_t domain1_state;
    static ec_slave_config_t *slave_config;
    static ec_slave_config_state_t slave_config_state;    

    /* Process Data */
    static uint8_t *domain1_pd;
    static unsigned int counter;
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
    // PDOs
    static const ec_pdo_entry_reg_t domain1_regs[];
    static ec_pdo_entry_info_t csp_pdo_entries[];
    static ec_pdo_info_t csp_pdos[];
    static ec_sync_info_t maxon_epos4_syncs_csp[];

    /* logging variables */
    bool is_operational;
    float t;
    std::vector<float> t1_array, velocity_input_array, position_input_array;
    std::vector<int32_t> velocity_output_array;
    std::vector<float> position_output_array;

    /* trajectory variables */
    float pos[2], vel[2], acc[2], moveTime[2];
    float pos_t, vel_t, acc_t;

    void initMaster();
    void checkDomainState();
    void checkMasterState();
    void checkSlaveConfigState();
    void getTrajectory(float q0, float q1, float v0, float v1, float a0, float a1, float t0, float t1);

public:
    int dataNum;

    CSPMotorController();
    ~CSPMotorController();
    void initTrajectoryParams();
    void cyclicTask();
    void saveData(const std::string &position_filename, const std::string &velocity_filename);
};

#endif // CSP_MOTOR_CONTROLLER_H
