#include "logger.hpp"

Logger::Logger(ros::NodeHandle nh)
{
    this->logfile.open("/home/csrobot/Desktop/Test_Logs/data.csv", std::ios_base::app);
}

Logger::~Logger()
{
    this->logfile.close();
}

void Logger::startTime()
{
    this->begin = ros::Time::now().toSec();
}

void Logger::endTime()
{
    this->end = ros::Time::now().toSec();
}

void Logger::resetTime()
{
    this->begin = 0;
    this->end = 0;
}

double Logger::getDuration()
{
    return this->end - this->begin;
    this->resetTime();
}


void Logger::writeHeaders()
{
    this->logfile << "Planning Success?,"
                  << "Grasp Score,"
                  << "# Grasp Attempts,"
                  << "Max Grasp Attempts,"
                  << "Time: Home -> Pick (Nav),"
                  << "Time: Pick -> Retract (Man),"
                  << "Time: Retract -> Place (Nav),"
                  << "Time: Place -> Dropped (Man),"
                  << "Time: Dropped -> Home (Nav),"
                  << '\n';
}

void Logger::writeData()
{
    this->logfile << this->data.planning_success
                  << ',' << this->data.score
                  << ',' << this->data.grasp_attempts
                  << ',' << this->data.max_attempts
                  << ',' << this->data.time_home_pick
                  << ',' << this->data.time_pick_retract
                  << ',' << this->data.time_retract_place
                  << ',' << this->data.time_place_dropped
                  << ',' << this->data.time_dropped_home
                  << '\n';
    this->data = Data();

}