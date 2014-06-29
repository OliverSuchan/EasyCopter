#ifndef __MAIN_WINDOW_H
#define __MAIN_WINDOW_H

#include <QtGui>
#include <QtGui/QMainWindow>
#include <QKeyEvent>
#include <QCloseEvent>
#include <QObject>
#include <iostream>
#include <vector>
#include <tuple>
#include <thread>
#include <future>
#include <opencv2/imgproc/imgproc.hpp>
#include "flightcontroller.hpp"
#include "ui_main_window.h"
#include "face_recognition.hpp"
#include "image_converter.hpp"


/**
 * @brief Das Hauptfenster<BR>
 * Bietet Steuerungsmöglichkeiten der Drohne und<BR>
 * deren Einstellungsmöglichkeiten
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    /**
     * @brief Standard-Konstruktor zum Erzeugen einer MainWindow-Instanz
     * @param p_Parent "Elternteil" der MainWindow-Instanz
     */
    MainWindow(QWidget *p_Parent = 0);

    /**
      * @brief Standard-Destruktor
      */
    ~MainWindow();

protected:

    /**
     * @brief Wird aufgerufen, sobald eine Taste auf der Tastur gedrückt wird.<BR>
     * (Zur Steuerung der Drohne)
     * @param p_KeyEvent Tasten-Event
     */
    void keyPressEvent(QKeyEvent *p_KeyEvent);

    /**
     * @brief Wird aufgerufen, wenn das Hauptfenster geschlossen wird
     * @param p_CloseEvent Schließen-Event
     */
    void closeEvent(QCloseEvent *p_CloseEvent);

private slots:

    /**
     * @brief Slot der aufgerufen wird,<BR>
     * wenn in den Kamerabildern der Drohne ein Gesicht erkannt wird.<BR>
     * Dieses Gesicht wird dann mit vorhandenen Gesichtern verglichen,<BR>
     * und fügt es hinzu, wenn keine Gemeinsamkeiten zu den anderen Bildern gefunden wurden
     * @param p_DetectedFace Detektiertes Gesicht
     */
    void onAddDetectedFace(cv::Mat p_DetectedFace);

    /**
     * @brief Wird aufgerufen, sobald ein oder mehrere Gesichter ausgewählt werden
     * @param p_ListItem Ausgewähltes Gesicht
     */
    void onSelectFace(QListWidgetItem *p_ListItem);

    /**
     * @brief Wird aufgerufen, sobald der Button zum Aktivieren,<BR>
     * beziehungsweise Deaktivieren, gedrückt wird und schaltet<BR>
     * den Modus um
     */
    void on_pushButton_3_clicked();

    /**
     * @brief Wird ausgeführt, wenn der Button "Starten",<BR>
     * zur Erzeugung der "Kamerabilder-Verarbeitungs-Instanz",<BR>
     * gedrückt wird und öffnet ein Fenster zur Visualisierung<BR>
     * der Kamerabilder
     */
    void on_pushButton_4_clicked();

    /**
     * @brief Sollte der Button "Verbinden" gedrückt werden,<BR>
     * so wird diese Funktion aufgerufen und überprüft, ob<BR>
     * eine Verbindung zum ROS-Server besteht
     */
    void on_button_connect_clicked();

private:

    /**
     * @brief Instanz zum Ansteuern der einzelnen Elemente,<BR>
     * die sich auf dem Hauptfenster befinden
     */
    Ui::MainWindowDesign m_Ui;

    /**
     * @brief Die detektierten Gesichter werden temporär gespeichert
     */
    std::vector<cv::Mat> m_TempImages;

    /**
     * @brief ImageConverter-Instanz, welche die Verarbeitung der Kamerabilder übernimmt
     * @see ImageConverter
     */
    ImageConverter *m_ImageConverter;

};


#else

class MainWindow;

#endif //__MAIN_WINDOW_H
