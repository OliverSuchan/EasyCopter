#include "../include/EasyCopter/flightcontroller.hpp"

Globals::Globals()
    : m_dLinearAcceleration(1.0),
      m_dAngularAcceleration(1.0),
      m_dActivateFaceDetection(false)
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


Globals &Globals::getInstance()
{
    static Globals gloInstance;
    return gloInstance;
}
