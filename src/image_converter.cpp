#include "../include/EasyCopter/image_converter.hpp"


void ImageConverter::init(int p_Argc, char** p_Argv)
{
    ros::init(p_Argc, p_Argv, "image_converter");
    ros::NodeHandle nodeHandle = ros::NodeHandle();
    image_transport::ImageTransport imageTransport = image_transport::ImageTransport(nodeHandle);
    imageSubscription = imageTransport.subscribe("/ardrone/image_raw", 1, &ImageConverter::imageUpdate, this);
    ros::spin();
}

ImageConverter::ImageConverter(int p_Argc, char ** p_Argv)
{
    cv::namedWindow(OPENCV_WINDOW, CV_WINDOW_KEEPRATIO);
    if(DEBUG_MODE)
    {
        //capture = cvCaptureFromCAM(-1);
        m_Capture = cvCaptureFromFile("/home/owley/Downloads/Who the FK is LeFloid.mp4");
    }
    faceCascade.load(faceCascadeName);
    eyesCascade.load(eyesCascadeName);
    init(p_Argc, p_Argv);
}

ImageConverter::~ImageConverter()
{
    shutdown();
}

void ImageConverter::shutdown()
{
    imageSubscription.shutdown();
    cv::destroyWindow(OPENCV_WINDOW);
}

cv::Mat ImageConverter::detectAndDisplay(cv::Mat p_Frame)
{
    std::vector<cv::Rect> faces;
    cv::Mat frameGray;

    cv::cvtColor(p_Frame, frameGray, CV_BGR2GRAY );
    cv::equalizeHist(frameGray, frameGray);

    faceCascade.detectMultiScale(frameGray, faces, 1.1, 2, CV_HAAR_SCALE_IMAGE, cv::Size(30, 30));
    cv::Rect faceToMatch;

    for( size_t i = 0; i < faces.size(); i++ )
    {
        Globals::getInstance()->addDetectedFace(p_Frame(faces[i]));
        for(auto item : Globals::getInstance()->m_CurrentFaces)
        {
            if(item.data)
                if(FaceRecognition().matchImages(item, p_Frame(faces[i])))
                {
                    faceToMatch = faces[i];
                    break;
                }
        }
        cv::Point centerPoint(faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height * 0.5);
        cv::ellipse(p_Frame, centerPoint, cv::Size( faces[i].width * 0.5, faces[i].height * 0.5), 0, 0, 360, cv::Scalar(255, 0, 255), 4, 8, 0);

        cv::Mat faceGray = frameGray(faces[i]);
        std::vector<cv::Rect> eyes;
        eyesCascade.detectMultiScale( faceGray, eyes, 1.1, 2, 0 |CV_HAAR_SCALE_IMAGE, cv::Size(30, 30) );

        for( size_t j = 0; j < eyes.size(); j++ )
        {
            cv::Point centerPoint( faces[i].x + eyes[j].x + eyes[j].width * 0.5, faces[i].y + eyes[j].y + eyes[j].height * 0.5 );
            int radius = cvRound((eyes[j].width + eyes[j].height)*0.25);
            cv::circle(p_Frame, centerPoint, radius, cv::Scalar( 255, 0, 0 ), 4, 8, 0);
        }
    }
    if(Globals::getInstance()->m_ActivateFaceDetection)
    {
        double f  = 92 * 100 / 15;
        if(faceToMatch.height > 0 && faceToMatch.width > 0)
        {
            if(faceToMatch.x + faceToMatch.width / 2 < (frameGray.cols - faceToMatch.width) / 2)
            {
                if(DEBUG_MODE)
                    std::cout << "rechts drehen" << std::endl;
                geometry_msgs::Twist command;
                command.angular.z = -Globals::getInstance()->m_AngularAcceleration;
                FlightController::getInstance().publishCommand(command);
            }
            else
            {
                if(DEBUG_MODE)
                    std::cout << "links drehen" << std::endl;
                geometry_msgs::Twist command;
                command.angular.z = Globals::getInstance()->m_AngularAcceleration;
                FlightController::getInstance().publishCommand(command);
            }

            if(faceToMatch.y + faceToMatch.height / 2 < (frameGray.rows - faceToMatch.height) / 2)
            {
                if(DEBUG_MODE)
                    std::cout << "runter" << std::endl;
                geometry_msgs::Twist command;
                command.linear.z = -Globals::getInstance()->m_LinearAcceleration;
                FlightController::getInstance().publishCommand(command);
            }
            else
            {
                if(DEBUG_MODE)
                    std::cout << "hoch" << std::endl;
                geometry_msgs::Twist command;
                command.linear.z = Globals::getInstance()->m_LinearAcceleration;
                FlightController::getInstance().publishCommand(command);
            }

            double distance = 15 * f / faces[0].width;
            std::cout << distance << std::endl;
            if(distance >= 50)
            {
                if(DEBUG_MODE)
                    std::cout << "vorwärts" << std::endl;
                geometry_msgs::Twist command;
                command.linear.x = Globals::getInstance()->m_LinearAcceleration;
                FlightController::getInstance().publishCommand(command);
            }
            else if(distance <= 20)
            {
                if(DEBUG_MODE)
                    std::cout << "rückwärts" << std::endl;
                geometry_msgs::Twist command;
                command.linear.x = -Globals::getInstance()->m_LinearAcceleration;
                FlightController::getInstance().publishCommand(command);
            }
        }
    }
    return p_Frame;
}

void ImageConverter::imageUpdate(const sensor_msgs::ImageConstPtr &p_Message)
{
    cv_bridge::CvImagePtr imagePointer;
    try
    {
        imagePointer = cv_bridge::toCvCopy(p_Message, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat faceRecognition;
    if(DEBUG_MODE)
        faceRecognition = detectAndDisplay(cvQueryFrame(m_Capture));
    else
        faceRecognition = detectAndDisplay(imagePointer->image);
    cv::waitKey(3);
    cv::imshow(OPENCV_WINDOW, faceRecognition);
}
