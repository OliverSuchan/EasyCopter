#include "../include/EasyCopter/globals.hpp"

Globals::Globals()
    : m_dLinearAcceleration(1.0),
      m_dAngularAcceleration(1.0)
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
