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

void Globals::addDetectedFace(cv::Mat p_DetectedFace)
{
    emit addDetectedFaceSig(p_DetectedFace);
}
