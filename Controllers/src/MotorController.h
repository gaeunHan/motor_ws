#include <iostream>
#include <vector>
using namespace std;

class EPOS4Slave{
private:
    /* motor vars.  */
    float PULSE;
    float GEAR_RATIO;
    float CNT_PER_REVOLUTION;
    float CNT_PER_DEGREE; 

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
    EPOS4Slave(float pulse, float gear_ratio);
    ~EPOS4Slave();
    void setTrajectoryParam(float pos0, float pos1, float pos2, float t0, float motion_input_period, int mode);
    void setTrajectory(float tick);
    void setVel0();
    float getVelTick();
    float getPosTick();
    float getMoveTime();
    float getCntPerRevolution();
    void logging(float tick, int32_t actualVel, int32_t actualPos);
    void saveData(const string &position_filename, const string &velocity_filename);    
};