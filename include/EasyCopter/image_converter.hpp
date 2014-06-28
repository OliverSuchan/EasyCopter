#ifndef __IMAGE_CONVERTER_HPP
#define __IMAGE_CONVERTER_HPP

#include <QMainWindow>
#include <QWidget>
#include <QImage>
#include <QGraphicsView>
#include <QGraphicsPixmapItem>
#include <QPixmap>
#include <QResizeEvent>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <iostream>
#include <thread>
#include "flightcontroller.hpp"
#include "face_recognition.hpp"

#define DEBUG_MODE 0

static const std::string OPENCV_WINDOW = "Image window";
static std::string faceCascadeName = "haarcascade_frontalface_alt.xml";
static std::string eyesCascadeName = "haarcascade_eye_tree_eyeglasses.xml";
static cv::CascadeClassifier faceCascade;
static cv::CascadeClassifier eyesCascade;

class ImageConverter
{
  void init(int p_Argc, char** p_Argv);
  cv::Mat detectAndDisplay(cv::Mat p_Frame);
  void imageUpdate(const sensor_msgs::ImageConstPtr& p_Message);
  image_transport::Subscriber imageSubscription;
  CvCapture* m_Capture;

public:
  ImageConverter(int p_Argc, char ** p_Argv);
  ~ImageConverter();
  void shutdown();

};

#endif //__IMAGE_CONVERTER_HPP
