#include "../include/EasyCopter/main_window.hpp"


using namespace Qt;

MainWindow::MainWindow(QWidget *p_Parent)
    : QMainWindow(p_Parent)
{
    m_Ui.setupUi(this);
    m_Ui.groupBox_3->hide();
    this->setGeometry(this->geometry().x(), this->geometry().y(), m_Ui.dock_status->width(), m_Ui.dock_status->height());
    FlightController::getInstance().startThread();
    Globals::getInstance()->m_KeyEvents.push_back(std::make_tuple(Qt::Key_O,  [=](){ FlightController::getInstance().takeOff(); }));
    Globals::getInstance()->m_KeyEvents.push_back(std::make_tuple(Qt::Key_L, [=](){ FlightController::getInstance().land(); }));
    Globals::getInstance()->m_KeyEvents.push_back(std::make_tuple(Qt::Key_W, [=](){ geometry_msgs::Twist command;
                                                                                         command.linear.z = Globals::getInstance()->m_LinearAcceleration;
                                                                                         FlightController::getInstance().publishCommand(command);}));
    Globals::getInstance()->m_KeyEvents.push_back(std::make_tuple(Qt::Key_S, [=](){ geometry_msgs::Twist command;
                                                                                         command.linear.z = -Globals::getInstance()->m_LinearAcceleration;
                                                                                         FlightController::getInstance().publishCommand(command); }));
    Globals::getInstance()->m_KeyEvents.push_back(std::make_tuple(Qt::Key_A, [=](){ geometry_msgs::Twist command;
                                                                                         command.angular.z = Globals::getInstance()->m_AngularAcceleration;
                                                                                         FlightController::getInstance().publishCommand(command);}));
    Globals::getInstance()->m_KeyEvents.push_back(std::make_tuple(Qt::Key_D, [=](){ geometry_msgs::Twist command;
                                                                                         command.angular.z = -Globals::getInstance()->m_AngularAcceleration;
                                                                                         FlightController::getInstance().publishCommand(command);}));

    Globals::getInstance()->m_KeyEvents.push_back(std::make_tuple(Qt::Key_Up, [=](){ geometry_msgs::Twist command;
                                                                                         command.linear.x = Globals::getInstance()->m_LinearAcceleration;
                                                                                         FlightController::getInstance().publishCommand(command);}));
    Globals::getInstance()->m_KeyEvents.push_back(std::make_tuple(Qt::Key_Down, [=](){ geometry_msgs::Twist command;
                                                                                         command.linear.x = -Globals::getInstance()->m_LinearAcceleration;
                                                                                         FlightController::getInstance().publishCommand(command);}));
    Globals::getInstance()->m_KeyEvents.push_back(std::make_tuple(Qt::Key_Left, [=](){ geometry_msgs::Twist command;
                                                                                         command.linear.y = Globals::getInstance()->m_LinearAcceleration;
                                                                                         FlightController::getInstance().publishCommand(command);}));
    Globals::getInstance()->m_KeyEvents.push_back(std::make_tuple(Qt::Key_Right, [=](){ geometry_msgs::Twist command;
                                                                                         command.linear.y = -Globals::getInstance()->m_LinearAcceleration;
                                                                                         FlightController::getInstance().publishCommand(command);}));

    QObject::connect(Globals::getInstance(), SIGNAL(addDetectedFaceSig(cv::Mat)), this, SLOT(onAddDetectedFace(cv::Mat)));
    m_Ui.listWidget_2->setIconSize(QSize(128, 256));
    connect(m_Ui.listWidget_2, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(onSelectFace(QListWidgetItem*)));
}

MainWindow::~MainWindow()
{
    FlightController::getInstance().stopThread();
}

void MainWindow::keyPressEvent(QKeyEvent *p_KeyEvent)
{
    Globals::getInstance()->executeKey(p_KeyEvent->key());
}

void MainWindow::closeEvent(QCloseEvent *p_CloseEvent)
{
    m_ImageConverter->shutdown();
    QMainWindow::closeEvent(p_CloseEvent);
}

void MainWindow::onAddDetectedFace(cv::Mat p_DetectedFace)
{
    for(int i = 0; i < m_TempImages.size(); i++)
    {
        if(FaceRecognition().matchImages(p_DetectedFace, m_TempImages.at(i)))
            return;
    }
    m_TempImages.push_back(p_DetectedFace);
    QImage itemImage((const uchar*)p_DetectedFace.data, p_DetectedFace.rows, p_DetectedFace.cols, p_DetectedFace.step.p[0], QImage::Format_RGB888);
    m_Ui.listWidget_2->addItem(new QListWidgetItem(QIcon(QPixmap::fromImage(itemImage.rgbSwapped())), QString::number(m_Ui.listWidget_2->count() + 1)));
}

void MainWindow::onSelectFace(QListWidgetItem *p_ListItem)
{
    Globals::getInstance()->m_CurrentFaces.clear();
    for(auto aItem : m_Ui.listWidget_2->selectedItems())
        Globals::getInstance()->m_CurrentFaces.push_back(m_TempImages.at(aItem->text().toInt() - 1));
}

void MainWindow::on_pushButton_3_clicked()
{
    Globals::getInstance()->m_ActivateFaceDetection = !Globals::getInstance()->m_ActivateFaceDetection;
}

void MainWindow::on_pushButton_4_clicked()
{
    m_ImageConverter = new ImageConverter(0, NULL);
}

void MainWindow::on_button_connect_clicked()
{
    m_Ui.groupBox_3->show();
    if(FlightController::getInstance().isConnected())
    {
        this->m_Ui.button_connect->setEnabled(false);
        m_Ui.dock_status->hide();
    }
}
