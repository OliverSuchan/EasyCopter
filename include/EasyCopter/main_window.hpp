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

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(int p_Argc, char** p_Argv, QWidget *p_Parent = 0);
    ~MainWindow();

protected:
    void keyPressEvent(QKeyEvent *p_KeyEvent);
    void closeEvent(QCloseEvent *p_CloseEvent);

private slots:
    void on_checkbox_use_environment_stateChanged(int p_State);
    void onAddDetectedFace(cv::Mat p_DetectedFace);
    void onSelectFace(QListWidgetItem *p_ListItem);
    void on_pushButton_3_clicked();
    void on_pushButton_4_clicked();
    void on_button_connect_clicked();

private:
    Ui::MainWindowDesign m_Ui;
    void readSettings();
    void writeSettings();
    void showNoMasterMessage();
    std::vector<cv::Mat> m_TempImages;
    ImageConverter *m_ImageConverter;

};


#else

class MainWindow;

#endif //__MAIN_WINDOW_H
