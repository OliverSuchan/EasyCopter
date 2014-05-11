#ifndef __FLIGHT_CONTROLLER_H
#define __FLIGHT_CONTROLLER_H

#include <QString>
#include <thread>
#include <mutex>
#include <atomic>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include "globals.hpp"

class FlightController
{
private:
    ros::NodeHandle *m_prnhFlightControllerNodeHandle;
    ros::Publisher m_rpFlightControllerPublisher;
    ros::Publisher m_rpTakeOffPublisher;
    ros::Publisher m_rpLandPublisher;
    ros::Rate *m_prrLoopRate;
    std::mutex m_muThreadMutex;
    std::thread *m_pthrThread;
    std::atomic<bool> m_bStop;
    std::atomic<bool> m_bRunLastCommand;
    geometry_msgs::Twist m_gmtLastCommand;
    Globals::CommandType m_ctCurrentCommandType;
    int m_iFrequency;
    void run();

public:
    FlightController(int p_iArgc, char **p_ppcArgv);
    FlightController(FlightController const&) = delete;
    FlightController& operator =(FlightController const&) = delete;
    ~FlightController();
    bool isConnected();
    void publishCommand(geometry_msgs::Twist p_gmtCommand);
    void takeOff();
    void land();
    void startThread();
    void stopThread();
    void setAutoRepeat(int p_iFrequency);
    void setRunLastCommand(bool p_bRunLastCommand);
    void setCommandType(Globals::CommandType p_ctCommandType);
    Globals::CommandType getCommandType();

};

#else

class FlightController;

#endif //__FLIGHT_CONTROLLER_H
