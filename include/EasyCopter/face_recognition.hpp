#ifndef __FACE_RECOGNITION_H
#define __FACE_RECOGNITION_H

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

class FaceRecognition
{
public:
    std::vector<cv::KeyPoint> getKeyPoints(cv::Mat p_cmImage, int p_iMinHessian = 15);
    std::vector<cv::DMatch> getMatches(cv::Mat p_cmImagePrimary, cv::Mat p_cmImageSecondary, int p_iMinDist = 100, int p_iMaxDist = 0);
    bool matchImages(cv::Mat p_cmImagePrimary, cv::Mat p_cmImageScondary);

};

#endif //__FACE_RECOGNITION_H
