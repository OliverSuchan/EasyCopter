#ifndef __GLOBALS_H
#define __GLOBALS_H

#include <QObject>
#include <vector>
#include <tuple>
#include <functional>
#include <opencv2/imgproc/imgproc.hpp>

/**
 * @brief Singleton-Klasse, die allen Klassen,<BR>
 * die benötigten Attribute zur Verfügung stellt
 */
class Globals : public QObject
{
    Q_OBJECT

public:

    /**
     * @brief Verschiedenen Kommando-Arten.<BR>
     * INSERT:<BR>
     * - das letzte Kommando wird durch das neue Kommando ergänzt<BR>
     *   um die Möglichkeit zu bieten, mehre Kommandos gleichzeitig<BR>
     *   auszuführen<BR>
     * SUBSTITUTE:<BR>
     * - ersetzt das vorherige Kommando lediglich durch das neue
     */
    enum CommandType
    {
        INSERT,
        SUBSTITUTE
    };

    /**
     * @brief Liefert die Globals-Singleton-Instanz
     * @return Globals-Singleton-Instanz
     */
    static Globals *getInstance();

    /**
     * @brief Array bestehend aus allen verfügbaren Tastenkombinationen<BR>
     * und deren Wirken auf die Drohne
     */
    std::vector<std::tuple<int, std::function<void()>>> m_KeyEvents;

    /**
     * @brief Lineare Beschleunigung.<BR>
     * Standardmäßig: 1
     */
    double m_LinearAcceleration;

    /**
     * @brief "Kantige-Beschleunigung".<BR>
     * (Für Rotation der Drohne)<BR>
     * Standardmäßig: 1
     */
    double m_AngularAcceleration;

    /**
     * @brief Wahrheitswert der angibt,<BR>
     * ob das Gesichtstracking aktiv ist
     */
    bool m_ActivateFaceDetection;

    /**
     * @brief Alle bisher wahrgenommenen Gesichter
     */
    std::vector<cv::Mat> m_CurrentFaces;

    void executeKey(int keyToExecute);

    /**
     * @brief Fügt ein detektiertes Gesicht hinzu
     * @param p_DetectedFace Detektiertes Gesicht
     */
    void addDetectedFace(cv::Mat p_DetectedFace);

signals:

    /**
     * @brief Signal das emittiert wird, wenn ein Gesicht detektiert wird
     * @param p_DetectedFace Detetiertes Gesicht
     */
    void addDetectedFaceSig(cv::Mat p_DetectedFace);

private:

    /**
     * @brief Standard-Konstruktor<.BR>
     * Ist aufgrund der Singleton-Struktur private
     */
    Globals();

    /**
     * @brief Standard-Kopierkonstruktor.<BR>
     * private und aufgabenlos wegen des Singleton-Designspatterns
     */
    Globals(Globals const&);

    /**
     * @brief Überladener Zuweisungsoperator.<BR>
     * aufgabenlos und private durch die Singleton-Struktur
     * @return neu zugewiesene Globals-Instanz
     */
    Globals& operator=(Globals const&);

    /**
      * @brief Standard-Destruktor.<BR>
      * Besitzt keine Aufgabe und das Zugriffsattribut private<BR>
      * aufgrund des Singleton-Designpatterns
      */
    ~Globals();

};

#endif //__GLOBALS_H
