#include "../include/EasyCopter/face_recognition.hpp"


std::vector<cv::KeyPoint> FaceRecognition::getKeyPoints(cv::Mat p_Image, int p_MinHessian)
{
    cv::FastFeatureDetector detector(p_MinHessian);
    std::vector<cv::KeyPoint> keyPoints;
    detector.detect(p_Image, keyPoints);
    return keyPoints;
}

std::vector<cv::DMatch> FaceRecognition::getMatches(cv::Mat p_ImagePrimary, cv::Mat p_ImageSecondary, int p_MinDist, int p_MaxDist)
{
    cv::SurfDescriptorExtractor extractor;
    cv::Mat primaryDescriptor, secondaryDescriptor;

    std::vector<cv::KeyPoint> primaryKeyPoints = getKeyPoints(p_ImagePrimary), secondaryKeyPoints = getKeyPoints(p_ImageSecondary);
    extractor.compute(p_ImagePrimary, primaryKeyPoints, primaryDescriptor);
    extractor.compute(p_ImageSecondary, secondaryKeyPoints, secondaryDescriptor);

    cv::FlannBasedMatcher matcher;
    std::vector<cv::DMatch> matches;

    if(primaryDescriptor.data && secondaryDescriptor.data)
        matcher.match(primaryDescriptor, secondaryDescriptor, matches);
    else
        return matches;

    for( int i = 0; i < primaryDescriptor.rows; i++ )
    {
        double dist = matches[i].distance;
        if( dist < p_MinDist ) p_MinDist = dist;
        if( dist > p_MaxDist ) p_MaxDist = dist;
    }

    std::vector<cv::DMatch> goodMatches;
    for( int i = 0; i < primaryDescriptor.rows; i++ )
    {
        if(matches[i].distance <= 3 * p_MinDist)
            goodMatches.push_back(matches[i]);
    }
    return goodMatches;
}

bool FaceRecognition::matchImages(cv::Mat p_ImagePrimary, cv::Mat p_ImageScondary)
{
    std::vector<cv::KeyPoint> primaryKeyPoints = getKeyPoints(p_ImagePrimary);
    std::vector<cv::KeyPoint> secondaryKeyPoints = getKeyPoints(p_ImageScondary);

    std::vector<cv::Point2f> primaryPoints;
    std::vector<cv::Point2f> secondaryPoints;

    std::vector<cv::DMatch> goodMatches = getMatches(p_ImagePrimary, p_ImageScondary);
    if(!goodMatches.size() || goodMatches.size() < 4)
        return false;

    for( int i = 0; i < goodMatches.size(); i++ )
    {
        primaryPoints.push_back( primaryKeyPoints[ goodMatches[i].queryIdx ].pt );
        secondaryPoints.push_back( secondaryKeyPoints[ goodMatches[i].trainIdx ].pt );
    }

    cv::Mat homography;
    try
    {
        homography = cv::findHomography(primaryPoints, secondaryPoints, CV_RANSAC);
    }
    catch(std::exception)
    {
        return false;
    }
    return true;
}
