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
CSPMotorController::CSPMotorController() : counter(FREQUENCY), t(0.0), idx(0), is_operational(false) {
    initMaster();
}

CSPMotorController::~CSPMotorController() {
    ecrt_release_master(master);
}

void CSPMotorController::initMaster() {
    master = ecrt_request_master(0);
    domain1 = ecrt_master_create_domain(master);
    slave_config = ecrt_master_slave_config(master, 0, 0, MAXON_EPOS4_5A);
    configurePDOs();
    activateMaster();
}

void CSPMotorController::configurePDOs() {
    ec_pdo_entry_info_t csp_pdo_entries[] = {
        {0x6040, 0x00, 16}, {0x607A, 0x00, 32}, {0x60B0, 0x00, 32}, {0x60B2, 0x00, 16},
        {0x6060, 0x00, 8}, {0x60FE, 0x01, 32}, {0x6041, 0x00, 16}, {0x6064, 0x00, 32},
        {0x606C, 0x00, 32}, {0x6077, 0x00, 16}, {0x6061, 0x00, 8}, {0x60FD, 0x00, 32}, {0x603F, 0x00, 16}
    };

    ec_pdo_info_t csp_pdos[] = {
        {0x1600, 6, csp_pdo_entries}, {0x1A00, 7, csp_pdo_entries + 6}
    };

    ec_sync_info_t syncs[] = {
        {0, EC_DIR_OUTPUT, 0, nullptr}, {1, EC_DIR_INPUT, 1, csp_pdos}, {0xff}
    };

    if (ecrt_slave_config_pdos(slave_config, EC_END, syncs)) {
        throw std::runtime_error("Failed to configure PDOs.");
    }

    ec_pdo_entry_reg_t domain1_regs[] = {
        {0, 0, MAXON_EPOS4_5A, 0x6040, 0x00, &offset_control_word},
        {0, 0, MAXON_EPOS4_5A, 0x607A, 0x00, &offset_target_position},
        {0, 0, MAXON_EPOS4_5A, 0x60B0, 0x00, &offset_position_offset},
        {0, 0, MAXON_EPOS4_5A, 0x60B2, 0x00, &offset_torque_offset},
        {0, 0, MAXON_EPOS4_5A, 0x6060, 0x00, &offset_modes_of_operation},
        {0, 0, MAXON_EPOS4_5A, 0x60FE, 0x01, &offset_digital_outputs},
        {0, 0, MAXON_EPOS4_5A, 0x6041, 0x00, &offset_status_word},
        {0, 0, MAXON_EPOS4_5A, 0x6064, 0x00, &offset_position_actual_value},
        {0, 0, MAXON_EPOS4_5A, 0x606C, 0x00, &offset_velocity_actual_value},
        {0, 0, MAXON_EPOS4_5A, 0x6077, 0x00, &offset_torque_actual_value},
        {0, 0, MAXON_EPOS4_5A, 0x6061, 0x00, &offset_modes_of_operation_display},
        {0, 0, MAXON_EPOS4_5A, 0x60FD, 0x00, &offset_digital_inputs},
        {0, 0, MAXON_EPOS4_5A, 0x603F, 0x00, &offset_error_code}
    };

    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs)) {
        throw std::runtime_error("Failed to register PDO entries.");
    }
}

void CSPMotorController::activateMaster() {
    ecrt_master_activate(master);
    domain1_pd = ecrt_domain_data(domain1);
}

void CSPMotorController::checkDomainState() {
    ec_domain_state_t ds;
    ecrt_domain_state(domain1, &ds);
    if (ds.working_counter != domain1_state.working_counter || ds.wc_state != domain1_state.wc_state) {
        std::cout << "Domain State Change: WC " << ds.working_counter << ", State " << ds.wc_state << std::endl;
        domain1_state = ds;
    }
}

void CSPMotorController::checkMasterState() {
    ec_master_state_t ms;
    ecrt_master_state(master, &ms);
    if (ms.slaves_responding != master_state.slaves_responding || ms.al_states != master_state.al_states || ms.link_up != master_state.link_up) {
        std::cout << ms.slaves_responding << " slave(s), AL state: 0x" << std::hex << ms.al_states << ", Link " << (ms.link_up ? "up" : "down") << std::endl;
        master_state = ms;
    }
}

void CSPMotorController::checkSlaveConfigState() {
    ec_slave_config_state_t s;
    ecrt_slave_config_state(slave_config, &s);
    if (s.al_state != slave_config_state.al_state || s.online != slave_config_state.online || s.operational != slave_config_state.operational) {
        std::cout << "EPOS4: AL state 0x" << std::hex << s.al_state << ", " << (s.online ? "online" : "offline") << ", " << (s.operational ? "operational" : "not operational") << std::endl;
        slave_config_state = s;
    }
}

void CSPMotorController::getTrajectory(float q0, float q1, float v0, float v1, float a0, float a1, float t0, float t1, float& pos_t, float& vel_t, float& acc_t) {
    float T = t1 - t0;
    float b0 = q0;
    float b1 = v0;
    float b2 = 0.5 * a0;
    float b3 = (20 * (q1 - q0) - (8 * v1 + 12 * v0) * T - (3 * a0 - a1) * T * T) / (2 * T * T * T);
    float b4 = (-30 * (q1 - q0) + (14 * v1 + 16 * v0) * T + (3 * a0 - 2 * a1) * T * T) / (2 * T * T * T * T);
    float b5 = (12 * (q1 - q0) - 6 * (v1 + v0) * T + (a1 - a0) * T * T) / (2 * T * T * T * T * T);

    float dt = t - t0;
    pos_t = b0 + b1 * dt + b2 * pow(dt, 2) + b3 * pow(dt, 3) + b4 * pow(dt, 4) + b5 * pow(dt, 5);
    vel_t = b1 + 2 * b2 * dt + 3 * b3 * pow(dt, 2) + 4 * b4 * pow(dt, 3) + 5 * b5 * pow(dt, 4);
    acc_t = 2 * b2 + 6 * b3 * dt + 12 * b4 * pow(dt, 2) + 20 * b5 * pow(dt, 3);
}

void CSPMotorController::cyclicTask() {
    checkDomainState();
    checkMasterState();
    checkSlaveConfigState();

    // 1초마다 상태 확인
    if (counter) {
        counter--;
    } else { // 1초 주기
        counter = FREQUENCY;  // 1초 주기로 초기화
        checkDomainState();
        checkMasterState();
        checkSlaveConfigState();
    }

    if (is_operational) {
        float pos_t, vel_t, acc_t;
        getTrajectory(0.0, 180.0 * CNT_PER_DEGREE, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, pos_t, vel_t, acc_t);

        EC_WRITE_U32(domain1_pd + offset_target_position, pos_t);
        t1_array.push_back(t);
        velocity_input_array.push_back((vel_t / (4096.0 * 35.0)) * 60);
        position_input_array.push_back(pos_t * 360.0 / 4096.0 / 35.0);
        velocity_output_array.push_back(EC_READ_S32(domain1_pd + offset_velocity_actual_value) / 35.0);
        position_output_array.push_back(((float)EC_READ_S32(domain1_pd + offset_position_actual_value) * 360.0f) / 4096.0f / 35.0);

        t += 0.001;
        idx++;
    }

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
