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
#include "ui_opencv.h"
#include <iostream>

static const std::string OPENCV_WINDOW = "Image window";
static std::string face_cascade_name = "haarcascade_frontalface_alt.xml";
static std::string eyes_cascade_name = "haarcascade_eye_tree_eyeglasses.xml";
static cv::CascadeClassifier face_cascade;
static cv::CascadeClassifier eyes_cascade;

class ImageConverter : public QMainWindow
{
  Ui::OpenCVDesign ui;
  void init(int argc, char** argv);
  cv::Mat detectAndDisplay(cv::Mat frame);
  void imageCb(const sensor_msgs::ImageConstPtr& msg);

protected:
  void resizeEvent(QResizeEvent *p_pqreResizeEvent);

public:
  ImageConverter(int argc, char **argv, QWidget *parent = 0);
  ~ImageConverter();

};

#endif //__IMAGE_CONVERTER_HPP
