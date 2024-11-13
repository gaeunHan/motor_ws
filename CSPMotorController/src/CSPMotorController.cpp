// CSPMotorController.cpp

#include "CSPMotorController.h"
#include <iostream>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <sys/resource.h> // 리소스 제한 설정
#include <signal.h>       // 신호 처리
#include <sched.h>        // 스케줄링 설정
#include <sys/mman.h>     // 메모리 잠금
#include <time.h>         // 시간 관련 함수

// 생성자 - 멤버 변수 초기화
CSPMotorController::CSPMotorController() : counter(0), t(0.0), is_operational(false) {
    initEcatVar();
    initMaster();    
}

CSPMotorController::~CSPMotorController() {
    ecrt_release_master(master);
}

void CSPMotorController::initEcatVar() {
    *master = NULL;
    master_state = {};

    *domain1 = NULL;
    domain1_state = {};

    *slave_config = NULL;
    slave_config_state = {};
}

void CSPMotorController::initMaster() {
    master = ecrt_request_master(0);
    if (!master) 
    {
        fprintf(stderr, "Failed to create master.\n");
        return;
    }

    domain1 = ecrt_master_create_domain(master);
    if (!domain1) 
    {
        fprintf(stderr, "Failed to create domain.\n");
        return;
    }

    if (!(slave_config = ecrt_master_slave_config(master, 0, 0, MAXON_EPOS4_5A))) 
    {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }

    printf("Configuring PDOs...\n");
    configurePDOs();

    printf("Activating master...\n");
    if (ecrt_master_activate(master)) return;

    printf("Activating domain...\n");
    if (!(domain1_pd = ecrt_domain_data(domain1))) return;
}

void CSPMotorController::configurePDOs() {
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

    // MDP module CSV mapping
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
    static ec_sync_info_t maxon_epos4_syncs_csp[] = {
        { 0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE },
        { 1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE },
        { 2, EC_DIR_OUTPUT, 1, csp_pdos + 0, EC_WD_ENABLE },
        { 3, EC_DIR_INPUT,  1, csp_pdos + 1, EC_WD_DISABLE },
        { 0xff }
    };

    if (ecrt_slave_config_pdos(slave_config, EC_END, maxon_epos4_syncs_csp)) {
        throw std::runtime_error("Failed to configure PDOs.\n");
        return;
    }

    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) {
        throw std::runtime_error("Failed to register PDO entries.");
        return;
    }
}

void CSPMotorController::checkDomainState() {
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

void CSPMotorController::checkMasterState() {
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

void CSPMotorController::checkSlaveConfigState() {
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

void CSPMotorController::initTrajectoryParams(){
    std::cout << "Enter pos[0], pos[1]: ";
    std::cin >> pos[0] >> pos[1];
    pos[0] *= CNT_PER_DEGREE;
    pos[1] *= CNT_PER_DEGREE;

    std::cout << std::endl << "Enter vel[0], vel[1]: ";
    std::cin >> vel[0] >> vel[1];

    std::cout << std::endl << "Enter acc[0], acc[1]: ";
    std::cin >> acc[0] >> acc[1];

    std::cout << std::endl << "Enter moveTime[0], moveTime[1]: ";
    std::cin >> moveTime[0] >> moveTime[1];    

    std::cout << std::endl << "Trajectory parameters initialized." << std::endl;
}

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

// 1ms period
void cyclic_task_csv()
{
    // (마스터가) receive process data
    ecrt_master_receive(master); // 데이터 받아(queue에서 데이터그램 dispatch)
    ecrt_domain_process(domain1); // 정상적인 통신 이루어졌는지 확인, 도메인 상태 업뎃

    // check process data state - optional
    checkDomainState(); //

    if (counter) 
    {
        counter--;   
    } 
    else // 1sec period
    { 
        counter = FREQUENCY;  
        
        // check for master state
        checkMasterState();

        // check for slave configuration state(s)
        checkSlaveConfigState(); // ecrt_slave_config_state()

        // get statusword
        uint16_t status_word = EC_READ_U16(domain1_pd + offset_status_word);
        
        // write process data   
        uint16_t check_status_word = status_word & 0x006F; // check 0,1,2,3,5,6th bit only
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
                uint16_t error_code = EC_READ_U16(domain1_pd + offset_error_code);
                printf("Fault, error code is: 0x%04X\n", error_code);

                // get controlword
                uint16_t control_word = EC_READ_U16(domain1_pd + offset_control_word);

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
        t1_array.push_back(t);
        velocity_input_array.push_back((vel_t / (PULSE * GEAR_RATIO)) * 60); // [rpm]
        position_input_array.push_back(pos_t * 360.0 / PULSE / GEAR_RATIO);
        velocity_output_array.push_back(EC_READ_S32(domain1_pd + offset_velocity_actual_value) / GEAR_RATIO);
        position_output_array.push_back(((float)EC_READ_S32(domain1_pd + offset_position_actual_value) * 360.0f) / PULSE / GEAR_RATIO);
        t += 0.001; 
    } 
     
    // (마스터가) send process data
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);
}

void CSPMotorController::saveData(const std::string &position_filename, const std::string &velocity_filename) {
    std::ofstream pos_file(position_filename);
    std::ofstream vel_file(velocity_filename);

    for (size_t i = 0; i < t1_array.size(); ++i) {
        pos_file << t1_array[i] << " " << position_input_array[i] << " " << position_output_array[i] << "\n";
        vel_file << t1_array[i] << " " << velocity_input_array[i] << " " << velocity_output_array[i] << "\n";
    }

    std::cout << "values saved to file." << std::endl;

    pos_file.close();
    vel_file.close();
}

void* CSPMotorController::stopSignalHandler(void* arg) {
    char stopSignal;
    std::cout << "Enter 'q' to stop motor: ";
    std::cin >> stopSignal;
    if (stopSignal == 'q') {
        static_cast<CSPMotorController*>(arg)->is_operational = false;
    }
    return nullptr;
}
