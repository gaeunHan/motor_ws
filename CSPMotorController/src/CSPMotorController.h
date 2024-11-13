// CSPMotorController.h

#ifndef CSP_MOTOR_CONTROLLER_H
#define CSP_MOTOR_CONTROLLER_H

#include <vector>
#include <string>
#include "ecrt.h"

// EtherCAT 관련 상수 정의
#define MAXON_EPOS4_5A 0x000000fb, 0x61500000 // Vendor ID, Product Code
#define PERIOD_NS   (1000000)
#define NSEC_PER_SEC (1000000000)
#define FREQUENCY (NSEC_PER_SEC / PERIOD_NS)
#define CNT_PER_DEGREE 398.0

class CSPMotorController {
private:
    ec_master_t *master;
    ec_domain_t *domain1;
    ec_slave_config_t *slave_config;
    uint8_t *domain1_pd;

    // 상태 변수
    ec_master_state_t master_state;
    ec_domain_state_t domain1_state;
    ec_slave_config_state_t slave_config_state;

    // RxPDO 및 TxPDO의 오프셋 변수
    unsigned int offset_control_word;
    unsigned int offset_target_position;
    unsigned int offset_position_offset;
    unsigned int offset_torque_offset;
    unsigned int offset_modes_of_operation;
    unsigned int offset_digital_outputs;
    unsigned int offset_status_word;
    unsigned int offset_position_actual_value;
    unsigned int offset_velocity_actual_value;
    unsigned int offset_torque_actual_value;
    unsigned int offset_modes_of_operation_display;
    unsigned int offset_digital_inputs;
    unsigned int offset_error_code;

    bool is_operational;
    float t;
    int idx;
    unsigned int counter;  // 1초 주기를 위한 counter 변수
    std::vector<float> t1_array, velocity_input_array, position_input_array;
    std::vector<int32_t> velocity_output_array;
    std::vector<float> position_output_array;

    void initMaster();
    void configurePDOs();
    void activateMaster();
    void checkDomainState();
    void checkMasterState();
    void checkSlaveConfigState();
    void getTrajectory(float q0, float q1, float v0, float v1, float a0, float a1, float t0, float t1, float& pos_t, float& vel_t, float& acc_t);

public:
    CSPMotorController();
    ~CSPMotorController();
    void cyclicTask();
    void startLogging();
    void saveData(const std::string &position_filename, const std::string &velocity_filename);
    static void* stopSignalHandler(void* arg);
};

#endif // CSP_MOTOR_CONTROLLER_H
