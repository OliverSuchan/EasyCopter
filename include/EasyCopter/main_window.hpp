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
    MainWindow(int argc, char **argv, QWidget *parent = 0);
    ~MainWindow();

protected:
    void keyPressEvent(QKeyEvent *p_pqkeKeyEvent);
    void closeEvent(QCloseEvent *p_pqceCloseEvent);

private slots:
    void on_button_connect_clicked(bool check);
    void on_checkbox_use_environment_stateChanged(int state);
    void onAddDetectedFace(cv::Mat p_cmDetectedFace);
    void onSelectFace(QListWidgetItem *p_pqlwiItem);
    void on_pushButton_3_clicked();
    void on_pushButton_4_clicked();

private:
    Ui::MainWindowDesign ui;
    void readSettings();
    void writeSettings();
    void showNoMasterMessage();
    std::vector<cv::Mat> m_mpcmImages;
    ImageConverter *m_picImageConverter;

};


#else

class MainWindow;

#endif //__MAIN_WINDOW_H
