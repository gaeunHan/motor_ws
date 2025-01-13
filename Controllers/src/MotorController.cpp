#include <iostream>
#include <vector>
#include <fstream>
#include <cmath>
#include "MotorController.h"
#include "ecrt.h"
using namespace std;

enum Mode{CSP_ZERO_VEL, CSV_PREV_VEL, CSP_PREDICT};

// constructor
EPOS4Slave::EPOS4Slave(float pulse, float gear_ratio) : PULSE(pulse * 4), GEAR_RATIO(gear_ratio),
                                                        CNT_PER_REVOLUTION(pulse * 4 * gear_ratio),
                                                        CNT_PER_DEGREE((pulse * 4 * gear_ratio) / 360.0f) 
{
    pos[0] = 0.0f; pos[1] = 0.0f;
    vel[0] = 0.0f; vel[1] = 0.0f;
    acc[0] = 0.0f; acc[1] = 0.0f;
    moveTime[0] = 0.0f; moveTime[1] = 0.0f;

    pos_tick = 0.0f;
    vel_tick = 0.0f;
    acc_tick = 0.0f;

    t1_array.clear();
    velocity_input_array.clear();
    position_input_array.clear();
    velocity_output_array.clear();
    position_output_array.clear();
}

// deconstructor
EPOS4Slave::EPOS4Slave::~EPOS4Slave(){

}


void EPOS4Slave::setTrajectoryParam(float pos0, float pos1, float pos2, float t0, float motion_input_period, int mode){
    pos[0] = pos0 * CNT_PER_DEGREE;
    pos[1] = pos1 * CNT_PER_DEGREE;

    switch(mode){
        case CSP_ZERO_VEL:
            vel[0] = 0.0;
            vel[1] = 0.0;
            break;
        case CSV_PREV_VEL:
            vel[1] = ((pos1 - pos0) / motion_input_period) * CNT_PER_DEGREE;
            break;
        case CSP_PREDICT:
            vel[1] = ((pos2 - pos1) / motion_input_period) * CNT_PER_DEGREE;
            // if(abs(pos1) > abs(pos2)){
            //     vel[1] = 0.0;
            // }
            // else{
            //     vel[1] = ((pos2 - pos1) / motion_input_period) * CNT_PER_DEGREE;
            // }
            break;
    }

    acc[0] = 0;
    acc[1] = 0;

    moveTime[0] = t0;
    moveTime[1] = t0 + motion_input_period;
}

void EPOS4Slave::setTrajectory(float tick){
    // calc coefficients
    float b0, b1, b2, b3, b4, b5;
    float T = moveTime[1] - moveTime[0];
    float q0 = pos[0], q1 = pos[1], v0 = vel[0], v1 = vel[1], a0 = acc[0], a1 = acc[1];
    b0 = q0;
    b1 = v0;
    b2 = 0.5 * a0;
    b3 = (1.0 / (2 * T * T * T)) * (20 * (q1 - q0) - (8 * v1 + 12 * v0) * T - (3 * a0 - a1) * T * T);
    b4 = (1.0 / (2 * T * T * T * T)) * (-30 * (q1 - q0) + (14 * v1 + 16 * v0) * T + (3 * a0 - 2 * a1) * T * T);
    b5 = (1.0 / (2 * T * T * T * T * T)) * (12 * (q1 - q0) - 6 * (v1 + v0) * T + (a1 - a0) * T * T);

    // pos, vel, acc formula
    float currTime = tick;
    float dt = currTime - moveTime[0];

    pos_tick = b0 + b1*dt + b2*pow(dt,2) + b3*pow(dt,3) + b4*pow(dt,4) + b5*pow(dt,5); // [encoder cnt]

    vel_tick = b1 + 2*b2*dt + 3*b3*pow(dt,2) + 4*b4*pow(dt,3) + 5*b5*pow(dt,4); // [encoder cnt / sec]
    vel_tick = (vel_tick * 60.0) / CNT_PER_REVOLUTION * GEAR_RATIO; // convert in [rpm]

    acc_tick = 2*b2 + 6*b3*dt + 12*b4*pow(dt,2) + 20*b5*pow(dt,3);
    acc_tick = (acc_tick * 60.0) / CNT_PER_REVOLUTION * GEAR_RATIO; // convert in [rpm/s]

    jerk_tick = 6*b3 + 24*b4*dt + 60*b5*pow(dt,2);
    jerk_tick = (jerk_tick * 60.0) / CNT_PER_REVOLUTION * GEAR_RATIO; // convert in [rpm/s^2]

    // debugging
    // cout << "vel[0]: " << vel[0] << " vel[1]: " << vel[1] << endl;
    // cout << "T: " << T << endl;
    // cout << "q0: " << q0 << " q1: " << q1 << " v0: " << v0 << " v1: " << v1 << " a0: " << a0 << " a1: " << a1 << endl;
    // cout << "b0: " << b0 << " b1: " << b1 << "b2: " << b2 << "b3: " << b3 << " b4: " << b4 << " b5: " << b5 << endl;
    // cout << "dt = currTime: " << currTime << " - moveTime[0]: " << moveTime[0] << " = " << dt << endl;
    // cout << "pos_tick at currTime: " << pos_tick << " @ " << currTime << endl;
}

void EPOS4Slave::setVel0(){
    vel[0] = vel[1];
}

float EPOS4Slave::getVelTick(){
    return vel_tick;
}

float EPOS4Slave::getPosTick(){
    return pos_tick;
}

float EPOS4Slave::getMoveTime(){
    return moveTime[1] - moveTime[0];
}

float EPOS4Slave::getCntPerRevolution(){
    return CNT_PER_REVOLUTION;
}

void EPOS4Slave::logging(float tick, int32_t actualVel, int32_t actualPos){
    t1_array.push_back(tick);
    
    position_input_array.push_back((pos_tick * 360.0) / CNT_PER_REVOLUTION); // [deg]
    velocity_input_array.push_back(vel_tick / GEAR_RATIO); // [rpm]
    acceleration_input_array.push_back(acc_tick / GEAR_RATIO); // [rpm/s]
    jerk_input_array.push_back(jerk_tick / GEAR_RATIO); // [rpm/s^2]
    
    position_output_array.push_back((actualPos * 360.0f) / CNT_PER_REVOLUTION); 
    velocity_output_array.push_back(actualVel / GEAR_RATIO);
}

void EPOS4Slave::saveData(const string &position_filename, const string &velocity_filename, const string &acceleration_filename, const string &jerk_filename) {
    ofstream pos_file(position_filename);
    ofstream vel_file(velocity_filename);
    ofstream acc_file(acceleration_filename);
    ofstream jerk_file(jerk_filename);

    for (size_t i = 0; i < t1_array.size(); ++i) {
        pos_file << t1_array[i] << " " << position_input_array[i] << " " << position_output_array[i] << "\n";
        vel_file << t1_array[i] << " " << velocity_input_array[i] << " " << velocity_output_array[i] << "\n";
        acc_file << t1_array[i] << " " << acceleration_input_array[i] << "\n";
        jerk_file << t1_array[i] << " " << jerk_input_array[i] << "\n";
    }

    cout << "\n**values saved to file." << endl;

    pos_file.close();
    vel_file.close();
}