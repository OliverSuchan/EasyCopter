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
    std::vector<std::tuple<int, std::function<void()>>> m_KeyEvents;
    double m_LinearAcceleration;
    double m_AngularAcceleration;
    bool m_ActivateFaceDetection;
    std::vector<cv::Mat> m_CurrentFaces;
    void addDetectedFace(cv::Mat p_DetectedFace);

signals:
    void addDetectedFaceSig(cv::Mat p_DetectedFace);

private:
    Globals();
    Globals(Globals const&);
    Globals& operator=(Globals const&);
    ~Globals();

};

#endif //__GLOBALS_H
