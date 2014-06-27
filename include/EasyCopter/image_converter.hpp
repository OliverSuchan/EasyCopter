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

static const std::string OPENCV_WINDOW = "Image window";
static std::string face_cascade_name = "haarcascade_frontalface_alt.xml";
static std::string eyes_cascade_name = "haarcascade_eye_tree_eyeglasses.xml";
static cv::CascadeClassifier face_cascade;
static cv::CascadeClassifier eyes_cascade;

class ImageConverter
{
  void init(int argc, char** argv);
  cv::Mat detectAndDisplay(cv::Mat frame);
  void imageCb(const sensor_msgs::ImageConstPtr& msg);
  image_transport::Subscriber image_sub;

  CvCapture* capture;

public:
  ImageConverter(int argc, char **argv);
  ~ImageConverter();
  void shutdown();

};

#endif //__IMAGE_CONVERTER_HPP
