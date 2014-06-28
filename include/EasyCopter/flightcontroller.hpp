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
    ros::NodeHandle *m_FlightControllerNodeHandle;
    ros::Publisher m_FlightControllerPublisher;
    ros::Publisher m_TakeOffPublisher;
    ros::Publisher m_LandPublisher;
    ros::Rate *m_LoopRate;
    std::mutex m_ThreadMutex;
    std::thread *m_Thread;
    std::atomic<bool> m_Stop;
    std::atomic<bool> m_RunLastCommand;
    geometry_msgs::Twist m_LastCommand;
    Globals::CommandType m_CurrentCommandType;
    int m_Frequency;
    void run();
    FlightController(int p_Argc, char **p_Argv);
    FlightController(FlightController const&) = delete;
    FlightController& operator =(FlightController const&) = delete;
    ~FlightController();

public:

    bool isConnected();
    void publishCommand(geometry_msgs::Twist p_Command);
    void takeOff();
    void land();
    void startThread();
    void stopThread();
    void setAutoRepeat(int p_Frequency);
    void setRunLastCommand(bool p_RunLastCommand);
    void setCommandType(Globals::CommandType p_CommandType);
    Globals::CommandType getCommandType();
    static FlightController &getInstance();

};

#endif //__FLIGHT_CONTROLLER_H
