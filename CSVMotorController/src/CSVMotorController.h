#include <iostream>
#include <vector>
using namespace std;

class EPOS4Slave{
private:
    /* trajectory variables */
    float pos[2], vel[2], acc[2], moveTime[2];
    float pos_tick, vel_tick, acc_tick;

    /* logging vars. */
    vector<float> t1_array;
    vector<float> velocity_input_array;
    vector<float> position_input_array;
    vector<float> velocity_output_array;
    vector<float> position_output_array;

public:
    EPOS4Slave();
    ~EPOS4Slave();
    void setTrajectoryParam(float pos0, float pos1, float vel0, float vel1, float acc0, float acc1, float t0, float t1);
    void setTrajectory(float tick);
    float getVelTick();
    float getPosTick();
    void logging(int tick, uint32_t actualVel, uint32_t actualPos);
    void saveData(const string &position_filename, const string &velocity_filename);    
};