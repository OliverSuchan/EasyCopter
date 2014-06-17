#include "../include/EasyCopter/image_converter.hpp"


void ImageConverter::init(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    ros::NodeHandle nh = ros::NodeHandle();
    image_transport::ImageTransport it = image_transport::ImageTransport(nh);
    image_transport::Subscriber image_sub = it.subscribe("/ardrone/image_raw", 1, &ImageConverter::imageCb, this);
    ros::spin();
}

ImageConverter::ImageConverter(int argc, char **argv)
{
    cv::namedWindow("OPENCV");
    //capture = cvCaptureFromCAM(-1);
    face_cascade.load( face_cascade_name );
    eyes_cascade.load( eyes_cascade_name );
    init(argc, argv);
}

ImageConverter::~ImageConverter()
{
    cv::destroyWindow("OPENCV");
}

cv::Mat ImageConverter::detectAndDisplay( cv::Mat frame )
{
    std::vector<cv::Rect> faces;
    cv::Mat frame_gray;

    cv::cvtColor( frame, frame_gray, CV_BGR2GRAY );
    cv::equalizeHist( frame_gray, frame_gray );

    face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(30, 30) );

    for( size_t i = 0; i < faces.size(); i++ )
    {
        cv::Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
        cv::ellipse( frame, center, cv::Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, cv::Scalar( 255, 0, 255 ), 4, 8, 0 );

        cv::Mat faceROI = frame_gray( faces[i] );
        std::vector<cv::Rect> eyes;
        eyes_cascade.detectMultiScale( faceROI, eyes, 1.1, 2, 0 |CV_HAAR_SCALE_IMAGE, cv::Size(30, 30) );

        for( size_t j = 0; j < eyes.size(); j++ )
        {
            cv::Point center( faces[i].x + eyes[j].x + eyes[j].width*0.5, faces[i].y + eyes[j].y + eyes[j].height*0.5 );
            int radius = cvRound( (eyes[j].width + eyes[j].height)*0.25 );
            cv::circle( frame, center, radius, cv::Scalar( 255, 0, 0 ), 4, 8, 0 );
        }
    }
    if(Globals::getInstance().m_dActivateFaceDetection)
    {
        double f  = 92 * 100 / 15;
        if(faces.size() >= 1)
        {
            if(faces[0].x + faces[0].width / 2 < (frame_gray.cols - faces[0].width) / 2)
            {
                std::cout << "links drehen" << std::endl;
                geometry_msgs::Twist command;
                command.angular.z = Globals::getInstance().m_dAngularAcceleration;
                FlightController::getInstance().publishCommand(command);
            }
            else
            {
                std::cout << "rechts drehen" << std::endl;
                geometry_msgs::Twist command;
                command.angular.z = -Globals::getInstance().m_dAngularAcceleration;
                FlightController::getInstance().publishCommand(command);
            }

            if(faces[0].y + faces[0].height / 2 < (frame_gray.rows - faces[0].height) / 2)
            {
                std::cout << "runter" << std::endl;
                geometry_msgs::Twist command;
                command.linear.z = -Globals::getInstance().m_dLinearAcceleration;
                FlightController::getInstance().publishCommand(command);
            }
            else
            {
                std::cout << "hoch" << std::endl;
                geometry_msgs::Twist command;
                command.linear.z = Globals::getInstance().m_dLinearAcceleration;
                FlightController::getInstance().publishCommand(command);
            }

            double distance = 15 * f / faces[0].width;
            std::cout << distance << std::endl;
            if(distance >= 50)
            {
                std::cout << "vorwärts" << std::endl;
                geometry_msgs::Twist command;
                command.linear.x = Globals::getInstance().m_dLinearAcceleration;
                FlightController::getInstance().publishCommand(command);
            }
            else if(distance <= 20)
            {
                std::cout << "rückwärts" << std::endl;
                geometry_msgs::Twist command;
                command.linear.x = -Globals::getInstance().m_dLinearAcceleration;
                FlightController::getInstance().publishCommand(command);
            }

        }
    }
    return frame;
}

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //cv::Mat faceRecognition = detectAndDisplay(cvQueryFrame(capture));
    cv::Mat faceRecognition = detectAndDisplay(cv_ptr->image);
    cv::waitKey(3);
    cv::imshow("OPENCV", faceRecognition);
}
