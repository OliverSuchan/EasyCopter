#ifndef __GLOBALS_H
#define __GLOBALS_H

#include <vector>
#include <tuple>
#include <functional>

class FlightController;

class Globals
{
public:
    enum CommandType
    {
        INSERT,
        SUBSTITUTE
    };

    static Globals &getInstance();
    std::vector<std::tuple<int, std::function<void()>>> m_rgtpivpKeyEvents;
    double m_dLinearAcceleration;
    double m_dAngularAcceleration;
    bool m_dActivateFaceDetection;

private:
    Globals();
    Globals(Globals const&);
    Globals& operator=(Globals const&);
    ~Globals();

};

#endif //__GLOBALS_H
