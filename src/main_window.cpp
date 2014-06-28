#include "../include/EasyCopter/main_window.hpp"


using namespace Qt;

MainWindow::MainWindow(int p_Argc, char** p_Argv, QWidget *p_Parent)
    : QMainWindow(p_Parent)
{
    m_Ui.setupUi(this);
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

    QObject::connect(m_Ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));
    readSettings();
    setWindowIcon(QIcon(":/images/icon.png"));
    if ( m_Ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked();
    }
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
    for(size_t index = 0; index < Globals::getInstance()->m_KeyEvents.size(); index++)
    {
        if(std::get<0>(Globals::getInstance()->m_KeyEvents.at(index)) == p_KeyEvent->key())
        {
            std::get<1>(Globals::getInstance()->m_KeyEvents.at(index))();

        }
    }
}

void MainWindow::showNoMasterMessage() {
    QMessageBox msgBox;
    msgBox.setText("Couldn't find the ros master.");
    msgBox.exec();
    close();
}

void MainWindow::on_checkbox_use_environment_stateChanged(int p_State) {
    bool enabled;
    if ( p_State == 0 ) {
        enabled = true;
    } else {
        enabled = false;
    }
    m_Ui.line_edit_master->setEnabled(enabled);
    m_Ui.line_edit_host->setEnabled(enabled);
}

void MainWindow::readSettings() {
    QSettings settings("Qt-Ros Package", "EasyCopter");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://localhost:11311/")).toString();
    QString host_url = settings.value("host_url", QString("localhost")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    m_Ui.line_edit_master->setText(master_url);
    m_Ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    m_Ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    m_Ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
        m_Ui.line_edit_master->setEnabled(false);
        m_Ui.line_edit_host->setEnabled(false);
        //ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::writeSettings() {
    QSettings settings("Qt-Ros Package", "EasyCopter");
    settings.setValue("master_url",m_Ui.line_edit_master->text());
    settings.setValue("host_url",m_Ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(m_Ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(m_Ui.checkbox_remember_settings->isChecked()));
}

void MainWindow::closeEvent(QCloseEvent *p_CloseEvent)
{
    writeSettings();
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
    if(FlightController::getInstance().isConnected())
    {
        this->m_Ui.button_connect->setEnabled(false);
    }
}
