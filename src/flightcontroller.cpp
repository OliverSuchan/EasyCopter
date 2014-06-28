#include "../include/EasyCopter/flightcontroller.hpp"

void FlightController::run()
{
    std::cerr << "Started Thread" << std::endl;
    while(!m_Stop.load() && m_FlightControllerNodeHandle->ok())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(m_Frequency));
        if(m_RunLastCommand.load())
            publishCommand(m_LastCommand);
        else
            publishCommand(geometry_msgs::Twist());
    }

    if(m_FlightControllerNodeHandle->ok()) ros::shutdown();
}

FlightController::FlightController(int p_Argc, char **p_Argv)
{
    ros::init(p_Argc, p_Argv, "FlightController");
    m_CurrentCommandType = Globals::INSERT;
}

FlightController::~FlightController()
{
    try
    {
        stopThread();
    }
    catch(std::exception &e)
    {
        std::cerr << e.what() << std::endl;
    }
}

bool FlightController::isConnected()
{
    return ros::ok();
}

void FlightController::publishCommand(geometry_msgs::Twist p_Command)
{
    m_ThreadMutex.lock();
    geometry_msgs::Twist CommandToPublish = p_Command;
    if(m_CurrentCommandType == Globals::INSERT)
    {
        CommandToPublish.linear.x = p_Command.linear.x + m_LastCommand.linear.x;
        CommandToPublish.linear.y = p_Command.linear.y + m_LastCommand.linear.y;
        CommandToPublish.linear.z = p_Command.linear.z + m_LastCommand.linear.z;
        CommandToPublish.angular.x = p_Command.angular.x + m_LastCommand.angular.x;
        CommandToPublish.angular.y = p_Command.angular.y + m_LastCommand.angular.y;
        CommandToPublish.angular.z = p_Command.angular.z + m_LastCommand.angular.z;
    }
    m_FlightControllerPublisher.publish(CommandToPublish);
    m_LastCommand = p_Command;
    m_ThreadMutex.unlock();
}


void FlightController::takeOff()
{
    m_ThreadMutex.lock();
    m_TakeOffPublisher.publish(std_msgs::Empty());
    m_ThreadMutex.unlock();
}

void FlightController::land()
{
    m_ThreadMutex.lock();
    m_LandPublisher.publish(std_msgs::Empty());
    m_ThreadMutex.unlock();
}

void FlightController::startThread()
{
    m_FlightControllerNodeHandle = new ros::NodeHandle();
    m_FlightControllerPublisher = m_FlightControllerNodeHandle->advertise<geometry_msgs::Twist>(m_FlightControllerNodeHandle->resolveName("cmd_vel"),1);
    m_TakeOffPublisher = m_FlightControllerNodeHandle->advertise<std_msgs::Empty>(m_FlightControllerNodeHandle->resolveName("ardrone/takeoff"),1);
    m_LandPublisher = m_FlightControllerNodeHandle->advertise<std_msgs::Empty>(m_FlightControllerNodeHandle->resolveName("ardrone/land"), 1);
    try
    {
        m_Stop.store(false);
        m_RunLastCommand.store(false);
        m_Frequency = 100;
        m_Thread = new std::thread(&FlightController::run, this);
    }
    catch(std::exception &e)
    {
        std::cerr << e.what() << std::endl;
    }
}

void FlightController::stopThread()
{
    m_Stop.store(true);
    m_Thread->join();
}

void FlightController::setAutoRepeat(int p_Frequency)
{
    m_ThreadMutex.lock();
    m_Frequency = p_Frequency;
    m_ThreadMutex.unlock();
}

void FlightController::setRunLastCommand(bool p_RunLastCommand)
{
    m_RunLastCommand.store(p_RunLastCommand);
}

void FlightController::setCommandType(Globals::CommandType p_CommandType)
{
    m_CurrentCommandType = p_CommandType;
}

Globals::CommandType FlightController::getCommandType()
{
    return m_CurrentCommandType;
}

FlightController &FlightController::getInstance()
{
    static FlightController FlightController(0, NULL);
    return FlightController;
}

