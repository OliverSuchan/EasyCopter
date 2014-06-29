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

/**
  * @brief Ist der Wert 1, so werden Ausgaben getätigt,<BR>
  * die den Verlauf des Programmes darstellen sollen
  */
#define DEBUG_MODE 0


/**
 * @brief Name des OpenCV-Fensters, indem<BR>
 * die Kamerabilder der Drohne angezeigt werden sollen
 */
static const std::string OPENCV_WINDOW = "Image window";

/**
 * @brief Dateiname für die "text-based"-Gesichtserkennung
 */
static std::string faceCascadeName = "haarcascade_frontalface_alt.xml";

/**
 * @brief Dateiname für die "text-based"-Augenerkennung
 */
static std::string eyesCascadeName = "haarcascade_eye_tree_eyeglasses.xml";

/**
 * @brief CascadeClassifier, der die geladene "text-based"-Gesichtserkennung,<BR>
 * in Form einer Klasse darstellt
 */
static cv::CascadeClassifier faceCascade;

/**
 * @brief CascadeClassifier, der die geladene "text-based"-Augenerkennung,<BR>
 * in Form einer Klasse darstellt
 */
static cv::CascadeClassifier eyesCascade;

/**
 * @brief Verarbeitet die eingehenden Kamerabilder der Drohne
 */
class ImageConverter
{

    /**
     * @brief Initiiert die Verbindung zu ROS
     * @param p_Argc Anzahl der Argument
     * @param p_Argv Argumente
     */
    void init(int p_Argc, char** p_Argv);

    /**
     * @brief Erkennt Gesichter und markiert diese<BR>
     * auf den einzelnen eingehenden Kamerabildern der Drohne<BR>
     * (inklusive Augen)
     * @param p_Frame Kamerabild der Drohne
     * @return bearbeitetes Kamerabild, mit detektierten Gesichtern und Augen
     */
    cv::Mat detectAndDisplay(cv::Mat p_Frame);

    /**
     * @brief Wird aufgerufen, sobald sich das Kamerabild ändert
     * @param p_Message Kamerabild
     */
    void imageUpdate(const sensor_msgs::ImageConstPtr& p_Message);

    /**
     * @brief Abonnement für das Kamerabild
     */
    image_transport::Subscriber imageSubscription;

    /**
     * @brief Wird für den "DEBUG_MODE" benötigt.<BR>
     * Wird verwendet um ein Video zu laden, was dann bearbeitet wird
     */
    CvCapture* m_Capture;

public:

    /**
     * @brief Konstruktor zum Erzeugen einer ImageConverter-Instanz
     * @param p_Argc Anzahl der Argumente
     * @param p_Argv Argumente
     */
    ImageConverter(int p_Argc, char ** p_Argv);

    /**
      * @brief Standard-Destruktor
      */
    ~ImageConverter();

    /**
     * @brief "Runterfahren" der ImageConverter-Instanz
     */
    void shutdown();

};

#endif //__IMAGE_CONVERTER_HPP
