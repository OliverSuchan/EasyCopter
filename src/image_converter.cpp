#include "../include/EasyCopter/image_converter.hpp"


void ImageConverter::init(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    ros::NodeHandle nh = ros::NodeHandle();
    image_transport::ImageTransport it = image_transport::ImageTransport(nh);
    image_transport::Subscriber image_sub = it.subscribe("/ardrone/image_raw", 1, &ImageConverter::imageCb, this);
    ros::spin();
}

void ImageConverter::resizeEvent(QResizeEvent *p_pqreResizeEvent)
{
    //this->ui.graphicsView->setGeometry(0, 0, p_pqreResizeEvent->size().width(), p_pqreResizeEvent->size().height());
}

ImageConverter::ImageConverter(int argc, char **argv, QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);
    this->show();
    face_cascade.load( face_cascade_name );
    eyes_cascade.load( eyes_cascade_name );
    init(argc, argv);
}

ImageConverter::~ImageConverter()
{

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

    QGraphicsScene scene;
    cv::Mat faceRecognition = detectAndDisplay(cv_ptr->image);
    QGraphicsPixmapItem item(QPixmap::fromImage(QImage((uchar*) faceRecognition.data, faceRecognition.cols, faceRecognition.rows, faceRecognition.step, QImage::Format_RGB888)));
    scene.addItem(&item);
    ui.graphicsView->setScene(&scene);
    ui.graphicsView->fitInView(scene.sceneRect(),Qt::KeepAspectRatio);
    cv::waitKey(3);
}
