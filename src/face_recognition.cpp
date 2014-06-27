#include "../include/EasyCopter/face_recognition.hpp"


std::vector<cv::KeyPoint> FaceRecognition::getKeyPoints(cv::Mat p_cvImage, int p_iMinHessian)
{
    cv::FastFeatureDetector detector(p_iMinHessian);
    std::vector<cv::KeyPoint> mpckpKeyPoints;
    detector.detect( p_cvImage, mpckpKeyPoints );
    return mpckpKeyPoints;
}

std::vector<cv::DMatch> FaceRecognition::getMatches(cv::Mat p_cvImagePrimary, cv::Mat p_cvImageSecondary, int p_iMinDist, int p_iMaxDist)
{
    cv::SurfDescriptorExtractor extractor;
    cv::Mat descriptors_1, descriptors_2;

    std::vector<cv::KeyPoint> keypoint_1 = getKeyPoints(p_cvImagePrimary);
    std::vector<cv::KeyPoint> keypoint_2 = getKeyPoints(p_cvImageSecondary);
    extractor.compute(p_cvImagePrimary, keypoint_1, descriptors_1);
    extractor.compute(p_cvImageSecondary, keypoint_2, descriptors_2);

    cv::FlannBasedMatcher matcher;
    std::vector<cv::DMatch> matches;

    if(descriptors_1.data && descriptors_2.data)
        matcher.match(descriptors_1, descriptors_2, matches);
    else
        return matches;

    for( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = matches[i].distance;
        if( dist < p_iMinDist ) p_iMinDist = dist;
        if( dist > p_iMaxDist ) p_iMaxDist = dist;
    }


    std::vector<cv::DMatch> good_matches;
    for( int i = 0; i < descriptors_1.rows; i++ )
    {
        if(matches[i].distance <= 3 * p_iMinDist)
            good_matches.push_back( matches[i]);
    }
    return good_matches;
}

bool FaceRecognition::matchImages(cv::Mat p_cvImagePrimary, cv::Mat p_cvImageScondary)
{
    std::vector<cv::KeyPoint> keypoints_object = getKeyPoints(p_cvImagePrimary);
    std::vector<cv::KeyPoint> keypoints_scene = getKeyPoints(p_cvImageScondary);

    std::vector<cv::Point2f> obj;
    std::vector<cv::Point2f> scene;

    cv::Mat img_matches;
    std::vector<cv::DMatch> good_matches = getMatches(p_cvImagePrimary, p_cvImageScondary);
    cv::drawMatches( p_cvImagePrimary, keypoints_object, p_cvImageScondary, keypoints_scene,
              good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
              std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    for( int i = 0; i < good_matches.size(); i++ )
    {
        obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
        scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
    }

    cv::Mat H;
    try
    {
        H = cv::findHomography(obj, scene, CV_RANSAC);
    }
    catch(std::exception)
    {
        return false;
    }
    return true;
}
