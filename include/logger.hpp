#include <iostream>
#include <fstream>
#include <ros/ros.h>

struct Data
{
    bool planning_success;
    float score = -1;
    long grasp_attempts = -1;
    long max_attempts = -1;
    float time_home_pick = -1;     // Nav
    float time_pick_retract = -1;  // Manip
    float time_retract_place = -1; // Nav
    float time_place_dropped = -1; // Manip
    float time_dropped_home = -1;  // Nav
};

class Logger
{
public:
    Logger(ros::NodeHandle nodeHandle);
    ~Logger();

    std::ofstream logfile;

    Data data;

    double begin;
    double end;

    void startTime();
    void endTime();
    void resetTime();
    double getDuration();
    void writeHeaders();
    void writeData();
};
