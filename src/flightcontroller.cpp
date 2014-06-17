#include "../include/EasyCopter/flightcontroller.hpp"

void FlightController::run()
{
    std::cerr << "Started Thread" << std::endl;
    while(!m_bStop.load() && m_prnhFlightControllerNodeHandle->ok())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(m_iFrequency));
        if(m_bRunLastCommand.load())
            publishCommand(m_gmtLastCommand);
        else
            publishCommand(geometry_msgs::Twist());
    }

    if(m_prnhFlightControllerNodeHandle->ok()) ros::shutdown();
}

FlightController::FlightController(int p_iArgc, char **p_ppcArgv)
{
    ros::init(p_iArgc, p_ppcArgv, "FlightController");
    m_ctCurrentCommandType = Globals::INSERT;
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

void FlightController::publishCommand(geometry_msgs::Twist p_gmtCommand)
{
    m_muThreadMutex.lock();
    geometry_msgs::Twist gmtCommandToPublish = p_gmtCommand;
    if(m_ctCurrentCommandType == Globals::INSERT)
    {
        gmtCommandToPublish.linear.x = p_gmtCommand.linear.x + m_gmtLastCommand.linear.x;
        gmtCommandToPublish.linear.y = p_gmtCommand.linear.y + m_gmtLastCommand.linear.y;
        gmtCommandToPublish.linear.z = p_gmtCommand.linear.z + m_gmtLastCommand.linear.z;
        gmtCommandToPublish.angular.x = p_gmtCommand.angular.x + m_gmtLastCommand.angular.x;
        gmtCommandToPublish.angular.y = p_gmtCommand.angular.y + m_gmtLastCommand.angular.y;
        gmtCommandToPublish.angular.z = p_gmtCommand.angular.z + m_gmtLastCommand.angular.z;
    }
    m_rpFlightControllerPublisher.publish(gmtCommandToPublish);
    m_gmtLastCommand = p_gmtCommand;
    m_muThreadMutex.unlock();
}


void FlightController::takeOff()
{
    m_muThreadMutex.lock();
    m_rpTakeOffPublisher.publish(std_msgs::Empty());
    m_muThreadMutex.unlock();
}

void FlightController::land()
{
    m_muThreadMutex.lock();
    m_rpLandPublisher.publish(std_msgs::Empty());
    m_muThreadMutex.unlock();
}

void FlightController::startThread()
{
    m_prnhFlightControllerNodeHandle = new ros::NodeHandle();
    m_rpFlightControllerPublisher = m_prnhFlightControllerNodeHandle->advertise<geometry_msgs::Twist>(m_prnhFlightControllerNodeHandle->resolveName("cmd_vel"),1);
    m_rpTakeOffPublisher = m_prnhFlightControllerNodeHandle->advertise<std_msgs::Empty>(m_prnhFlightControllerNodeHandle->resolveName("ardrone/takeoff"),1);
    m_rpLandPublisher = m_prnhFlightControllerNodeHandle->advertise<std_msgs::Empty>(m_prnhFlightControllerNodeHandle->resolveName("ardrone/land"), 1);
    try
    {
        m_bStop.store(false);
        m_bRunLastCommand.store(false);
        m_iFrequency = 100;
        m_pthrThread = new std::thread(&FlightController::run, this);
    }
    catch(std::exception &e)
    {
        std::cerr << e.what() << std::endl;
    }
}

void FlightController::stopThread()
{
    m_bStop.store(true);
    m_pthrThread->join();
}

void FlightController::setAutoRepeat(int p_iFrequency)
{
    m_muThreadMutex.lock();
    m_iFrequency = p_iFrequency;
    m_muThreadMutex.unlock();
}

void FlightController::setRunLastCommand(bool p_bRunLastCommand)
{
    m_bRunLastCommand.store(p_bRunLastCommand);
}

void FlightController::setCommandType(Globals::CommandType p_ctCommandType)
{
    m_ctCurrentCommandType = p_ctCommandType;
}

Globals::CommandType FlightController::getCommandType()
{
    return m_ctCurrentCommandType;
}

FlightController &FlightController::getInstance()
{
    static FlightController fcController(0, NULL);
    return fcController;
}

