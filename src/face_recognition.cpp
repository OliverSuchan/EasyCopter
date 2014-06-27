#include "../include/EasyCopter/face_recognition.hpp"


std::vector<cv::KeyPoint> FaceRecognition::getKeyPoints(cv::Mat p_cmImage, int p_iMinHessian)
{
    cv::FastFeatureDetector cffdDetector(p_iMinHessian);
    std::vector<cv::KeyPoint> mpckpKeyPoints;
    cffdDetector.detect( p_cmImage, mpckpKeyPoints );
    return mpckpKeyPoints;
}

std::vector<cv::DMatch> FaceRecognition::getMatches(cv::Mat p_cmImagePrimary, cv::Mat p_cmImageSecondary, int p_iMinDist, int p_iMaxDist)
{
    cv::SurfDescriptorExtractor csdeExtractor;
    cv::Mat cmPrimaryDescriptor, cmSecondaryDescriptor;

    std::vector<cv::KeyPoint> mpckpPrimaryKeyPoints = getKeyPoints(p_cmImagePrimary);
    std::vector<cv::KeyPoint> mpckpSecondaryKeyPoints = getKeyPoints(p_cmImageSecondary);
    csdeExtractor.compute(p_cmImagePrimary, mpckpPrimaryKeyPoints, cmPrimaryDescriptor);
    csdeExtractor.compute(p_cmImageSecondary, mpckpSecondaryKeyPoints, cmSecondaryDescriptor);

    cv::FlannBasedMatcher matcher;
    std::vector<cv::DMatch> matches;

    if(cmPrimaryDescriptor.data && cmSecondaryDescriptor.data)
        matcher.match(cmPrimaryDescriptor, cmSecondaryDescriptor, matches);
    else
        return matches;

    for( int i = 0; i < cmPrimaryDescriptor.rows; i++ )
    {
        double dDist = matches[i].distance;
        if( dDist < p_iMinDist ) p_iMinDist = dDist;
        if( dDist > p_iMaxDist ) p_iMaxDist = dDist;
    }

    std::vector<cv::DMatch> mpcdmGoodMatches;
    for( int i = 0; i < cmPrimaryDescriptor.rows; i++ )
    {
        if(matches[i].distance <= 3 * p_iMinDist)
            mpcdmGoodMatches.push_back( matches[i]);
    }
    return mpcdmGoodMatches;
}

bool FaceRecognition::matchImages(cv::Mat p_cmImagePrimary, cv::Mat p_cmImageScondary)
{
    std::vector<cv::KeyPoint> mpckpPrimaryKeyPoints = getKeyPoints(p_cmImagePrimary);
    std::vector<cv::KeyPoint> mpckpSecondaryKeyPoints = getKeyPoints(p_cmImageScondary);

    std::vector<cv::Point2f> mpcp2fPrimary;
    std::vector<cv::Point2f> mpcp2fSecondary;

    std::vector<cv::DMatch> mpcdmGoodMatches = getMatches(p_cmImagePrimary, p_cmImageScondary);
    if(!mpcdmGoodMatches.size() || mpcdmGoodMatches.size() < 4)
        return false;

    for( int i = 0; i < mpcdmGoodMatches.size(); i++ )
    {
        mpcp2fPrimary.push_back( mpckpPrimaryKeyPoints[ mpcdmGoodMatches[i].queryIdx ].pt );
        mpcp2fSecondary.push_back( mpckpSecondaryKeyPoints[ mpcdmGoodMatches[i].trainIdx ].pt );
    }

    cv::Mat cmHomography;
    try
    {
        cmHomography = cv::findHomography(mpcp2fPrimary, mpcp2fSecondary, CV_RANSAC);
    }
    catch(std::exception)
    {
        return false;
    }
    return true;
}
