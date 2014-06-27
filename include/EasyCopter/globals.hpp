#ifndef __GLOBALS_H
#define __GLOBALS_H

#include <QObject>
#include <vector>
#include <tuple>
#include <functional>
#include <opencv2/imgproc/imgproc.hpp>

class Globals : public QObject
{
    Q_OBJECT

public:
    enum CommandType
    {
        INSERT,
        SUBSTITUTE
    };

    static Globals *getInstance();
    std::vector<std::tuple<int, std::function<void()>>> m_rgtpivpKeyEvents;
    double m_dLinearAcceleration;
    double m_dAngularAcceleration;
    bool m_dActivateFaceDetection;
    std::vector<cv::Mat> m_cmCurrentFaces;
    void addDetectedFace(cv::Mat p_cmDetectedFace);

signals:
    void addDetectedFaceSig(cv::Mat p_cmDetectedFace);

private:
    Globals();
    Globals(Globals const&);
    Globals& operator=(Globals const&);
    ~Globals();

};

#endif //__GLOBALS_H
