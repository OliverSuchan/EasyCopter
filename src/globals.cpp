#include "../include/EasyCopter/flightcontroller.hpp"

Globals::Globals()
    : QObject(),
      m_LinearAcceleration(1.0),
      m_AngularAcceleration(1.0),
      m_ActivateFaceDetection(false)
{

}


Globals::Globals(const Globals &)
{

}


Globals &Globals::operator=(const Globals &)
{

}


Globals::~Globals()
{

}


Globals *Globals::getInstance()
{
    static Globals *GlobalsInstance = new Globals();
    return GlobalsInstance;
}

void Globals::executeKey(int keyToExecute)
{
    for(std::tuple<int, std::function<void()> > tupleItem : m_KeyEvents)
        if(std::get<0>(tupleItem) == keyToExecute)
            std::get<1>(tupleItem)();
}

void Globals::addDetectedFace(cv::Mat p_DetectedFace)
{
    emit addDetectedFaceSig(p_DetectedFace);
}
