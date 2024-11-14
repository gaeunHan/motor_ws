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
    void setTrajectoryParam();
    void setTrajectory(float tick);
    float getVelTick();
    float getPosTick();
    void logging(int tick, uint32 actualVel, uint32 actualPos);
    void saveData(const string &position_filename, const string &velocity_filename);    
};