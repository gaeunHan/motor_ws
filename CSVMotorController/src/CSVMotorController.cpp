#include <iostream>
#include <vector>
#include <fstream>
#include <cmath>
#include "CSVMotorController.h"
#include "ecrt.h"
using namespace std;

// constructor
EPOS4Slave::EPOS4Slave()(float pulse, float gear_ratio) : PULSE(pulse * 4.0), GEAR_RATIO(gear_ratio),
                                                        CNT_PER_REVOLUTION(pulse * gear_ratio),
                                                        CNT_PER_DEGREE((pulse * gear_ratio) / 360.0f) 
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


void EPOS4Slave::setTrajectoryParam(float pos0, float pos1, float vel0, float vel1, float acc0, float acc1, float t0, float t1){
    pos[0] = pos0 * CNT_PER_DEGREE;
    pos[1] = pos1 * CNT_PER_DEGREE;

    vel[0] = vel0;
    vel[1] = vel1;

    acc[0] = acc0;
    acc[1] = acc1;

    moveTime[0] = t0;
    moveTime[1] = t1;

    cout << endl << "Trajectory parameters initialized." << endl;
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
    vel_tick = ((vel_tick * 60.0) / CNT_PER_REVOLUTION) * 35.0; // convert it into [rpm]
    acc_tick = 2*b2 + 6*b3*dt + 12*b4*pow(dt,2) + 20*b5*pow(dt,3);
}

float EPOS4Slave::getVelTick(){
    return vel_tick;
}

float EPOS4Slave::getPosTick(){
    return pos_tick;
}

void EPOS4Slave::logging(float tick, uint32_t actualVel, uint32_t actualPos){
    t1_array.push_back(tick);
    velocity_input_array.push_back(vel_tick / GEAR_RATIO); // [rpm]
    position_input_array.push_back((pos[0] / CNT_PER_DEGREE) + (pos_tick * 360.0 / PULSE / GEAR_RATIO)); // [deg]
    velocity_output_array.push_back(actualVel / GEAR_RATIO); // actual velocity 읽을 때 기어비로 나눠줘야 함. 
    position_output_array.push_back(((actualPos * 360.0f) / PULSE) / GEAR_RATIO); // logging pos in degree 
}

void EPOS4Slave::saveData(const string &position_filename, const string &velocity_filename) {
    ofstream pos_file(position_filename);
    ofstream vel_file(velocity_filename);

    for (size_t i = 0; i < t1_array.size(); ++i) {
        pos_file << t1_array[i] << " " << position_input_array[i] << " " << position_output_array[i] << "\n";
        vel_file << t1_array[i] << " " << velocity_input_array[i] << " " << velocity_output_array[i] << "\n";
    }

    cout << "\n**values saved to file." << endl;

    pos_file.close();
    vel_file.close();
}