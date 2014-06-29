#ifndef __FACE_RECOGNITION_H
#define __FACE_RECOGNITION_H

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

/**
 * @brief Klasse zum Erkennen von Gesichtern
 */
class FaceRecognition
{
public:
    /**
     * @brief Liefert die interessanten Punkte eine Bildes
     * @param p_Image Bild dessen interessante Punkte gefunden werden sollen
     * @param p_MinHessian Mindest Dimension der Hesse-Matrix.<BR>
     * Je größer der Wert, desto geringer ist die Möglichkeit auf ein Ergebnis,<BR>
     * da diese Schwelle überwunden werden muss.
     * @return Vektor bestehend aus den interessanten Punkten des Bildes
     */
    std::vector<cv::KeyPoint> getKeyPoints(cv::Mat p_Image, int p_MinHessian = 15);

    /**
     * @brief Gibt Übereinstimmungen von interessanten Punkten<BR>
     * zweier Bilder zurück
     * @param p_ImagePrimary Bild #1
     * @param p_ImageSecondary Bild #2
     * @param p_MinDist Minimale Distanz
     * @param p_MaxDist Maximale Distanz
     * @return Vektor bestehend aus den Übereinstimmungen<BR>
     * der interessanten Punkte der beiden Bilder
     */
    std::vector<cv::DMatch> getMatches(cv::Mat p_ImagePrimary, cv::Mat p_ImageSecondary, int p_MinDist = 100, int p_MaxDist = 0);

    /**
     * @brief Gibt an, ob die Bilder übereinstimmen
     * @param p_ImagePrimary Bild #1
     * @param p_ImageScondary Bild #2
     * @return Wahrheitswert der angibt,<BR>
     * ob das eine Bild in dem anderen Bild gefunden werden konnte
     */
    bool matchImages(cv::Mat p_ImagePrimary, cv::Mat p_ImageScondary);

};

#endif //__FACE_RECOGNITION_H
