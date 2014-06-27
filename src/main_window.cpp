#include "../include/EasyCopter/main_window.hpp"


using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);
    FlightController::getInstance().startThread();
    Globals::getInstance()->m_rgtpivpKeyEvents.push_back(std::make_tuple(Qt::Key_O,  [=](){ FlightController::getInstance().takeOff(); }));
    Globals::getInstance()->m_rgtpivpKeyEvents.push_back(std::make_tuple(Qt::Key_L, [=](){ FlightController::getInstance().land(); }));
    Globals::getInstance()->m_rgtpivpKeyEvents.push_back(std::make_tuple(Qt::Key_W, [=](){ geometry_msgs::Twist command;
                                                                                         command.linear.z = Globals::getInstance()->m_dLinearAcceleration;
                                                                                         FlightController::getInstance().publishCommand(command);}));
    Globals::getInstance()->m_rgtpivpKeyEvents.push_back(std::make_tuple(Qt::Key_S, [=](){ geometry_msgs::Twist command;
                                                                                         command.linear.z = -Globals::getInstance()->m_dLinearAcceleration;
                                                                                         FlightController::getInstance().publishCommand(command); }));
    Globals::getInstance()->m_rgtpivpKeyEvents.push_back(std::make_tuple(Qt::Key_A, [=](){ geometry_msgs::Twist command;
                                                                                         command.angular.z = Globals::getInstance()->m_dAngularAcceleration;
                                                                                         FlightController::getInstance().publishCommand(command);}));
    Globals::getInstance()->m_rgtpivpKeyEvents.push_back(std::make_tuple(Qt::Key_D, [=](){ geometry_msgs::Twist command;
                                                                                         command.angular.z = -Globals::getInstance()->m_dAngularAcceleration;
                                                                                         FlightController::getInstance().publishCommand(command);}));

    Globals::getInstance()->m_rgtpivpKeyEvents.push_back(std::make_tuple(Qt::Key_Up, [=](){ geometry_msgs::Twist command;
                                                                                         command.linear.x = Globals::getInstance()->m_dLinearAcceleration;
                                                                                         FlightController::getInstance().publishCommand(command);}));
    Globals::getInstance()->m_rgtpivpKeyEvents.push_back(std::make_tuple(Qt::Key_Down, [=](){ geometry_msgs::Twist command;
                                                                                         command.linear.x = -Globals::getInstance()->m_dLinearAcceleration;
                                                                                         FlightController::getInstance().publishCommand(command);}));
    Globals::getInstance()->m_rgtpivpKeyEvents.push_back(std::make_tuple(Qt::Key_Left, [=](){ geometry_msgs::Twist command;
                                                                                         command.linear.y = Globals::getInstance()->m_dLinearAcceleration;
                                                                                         FlightController::getInstance().publishCommand(command);}));
    Globals::getInstance()->m_rgtpivpKeyEvents.push_back(std::make_tuple(Qt::Key_Right, [=](){ geometry_msgs::Twist command;
                                                                                         command.linear.y = -Globals::getInstance()->m_dLinearAcceleration;
                                                                                         FlightController::getInstance().publishCommand(command);}));

    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));
    readSettings();
    setWindowIcon(QIcon(":/images/icon.png"));
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }
    QObject::connect(Globals::getInstance(), SIGNAL(addDetectedFaceSig(cv::Mat)), this, SLOT(onAddDetectedFace(cv::Mat)));
    ui.listWidget_2->setIconSize(QSize(128, 256));
    connect(ui.listWidget_2, SIGNAL(itemClicked(QListWidgetItem*)), this, SLOT(onSelectFace(QListWidgetItem*)));
}

MainWindow::~MainWindow()
{
    FlightController::getInstance().stopThread();
}

void MainWindow::keyPressEvent(QKeyEvent *p_pqkeEvent)
{
    for(size_t stIndex = 0; stIndex < Globals::getInstance()->m_rgtpivpKeyEvents.size(); stIndex++)
    {
        if(std::get<0>(Globals::getInstance()->m_rgtpivpKeyEvents.at(stIndex)) == p_pqkeEvent->key())
        {
            std::get<1>(Globals::getInstance()->m_rgtpivpKeyEvents.at(stIndex))();

        }
    }
}

void MainWindow::showNoMasterMessage() {
    QMessageBox msgBox;
    msgBox.setText("Couldn't find the ros master.");
    msgBox.exec();
    close();
}

void MainWindow::on_button_connect_clicked(bool check ) {
    if(FlightController::getInstance().isConnected())
    {
        this->ui.button_connect->setEnabled(false);
    }
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
    bool enabled;
    if ( state == 0 ) {
        enabled = true;
    } else {
        enabled = false;
    }
    ui.line_edit_master->setEnabled(enabled);
    ui.line_edit_host->setEnabled(enabled);
}

void MainWindow::readSettings() {
    QSettings settings("Qt-Ros Package", "EasyCopter");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://localhost:11311/")).toString();
    QString host_url = settings.value("host_url", QString("localhost")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
        ui.line_edit_master->setEnabled(false);
        ui.line_edit_host->setEnabled(false);
        //ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::writeSettings() {
    QSettings settings("Qt-Ros Package", "EasyCopter");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    writeSettings();
    m_picImageConverter->shutdown();
    //cv::destroyAllWindows();
    QMainWindow::closeEvent(event);
}

void MainWindow::onAddDetectedFace(cv::Mat p_cmDetectedFace)
{
    for(int i = 0; i < m_mpcmImages.size(); i++)
    {
        if(FaceRecognition().matchImages(p_cmDetectedFace, m_mpcmImages.at(i)))
            return;
    }
    m_mpcmImages.push_back(p_cmDetectedFace);
    QImage qiImg((const uchar*)p_cmDetectedFace.data, p_cmDetectedFace.rows, p_cmDetectedFace.cols, p_cmDetectedFace.step.p[0], QImage::Format_RGB888);
    ui.listWidget_2->addItem(new QListWidgetItem(QIcon(QPixmap::fromImage(qiImg.rgbSwapped())), QString::number(ui.listWidget_2->count() + 1)));
}

void MainWindow::onSelectFace(QListWidgetItem *p_pqlwiItem)
{
    Globals::getInstance()->m_cmCurrentFace = m_mpcmImages.at(p_pqlwiItem->text().toInt() - 1);
}

void MainWindow::on_pushButton_3_clicked()
{
    Globals::getInstance()->m_dActivateFaceDetection = !Globals::getInstance()->m_dActivateFaceDetection;
}

void MainWindow::on_pushButton_4_clicked()
{
    m_picImageConverter = new ImageConverter(0, NULL);
}
