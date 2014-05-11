#include "../include/EasyCopter/main_window.hpp"


using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent),
      m_fcController(argc, argv)
{
    ui.setupUi(this);
    m_fcController.startThread();
    Globals::getInstance().m_rgtpivpKeyEvents.push_back(std::make_tuple(Qt::Key_O,  [=](){ m_fcController.takeOff(); }));
    Globals::getInstance().m_rgtpivpKeyEvents.push_back(std::make_tuple(Qt::Key_L, [=](){ m_fcController.land(); }));
    Globals::getInstance().m_rgtpivpKeyEvents.push_back(std::make_tuple(Qt::Key_W, [=](){ geometry_msgs::Twist command;
                                                                                         command.linear.z = Globals::getInstance().m_dLinearAcceleration;
                                                                                         m_fcController.publishCommand(command);}));
    Globals::getInstance().m_rgtpivpKeyEvents.push_back(std::make_tuple(Qt::Key_S, [=](){ geometry_msgs::Twist command;
                                                                                         command.linear.z = -Globals::getInstance().m_dLinearAcceleration;
                                                                                         m_fcController.publishCommand(command); }));
    Globals::getInstance().m_rgtpivpKeyEvents.push_back(std::make_tuple(Qt::Key_A, [=](){ geometry_msgs::Twist command;
                                                                                         command.angular.z = Globals::getInstance().m_dAngularAcceleration;
                                                                                         m_fcController.publishCommand(command);}));
    Globals::getInstance().m_rgtpivpKeyEvents.push_back(std::make_tuple(Qt::Key_D, [=](){ geometry_msgs::Twist command;
                                                                                         command.angular.z = -Globals::getInstance().m_dAngularAcceleration;
                                                                                         m_fcController.publishCommand(command);}));

    Globals::getInstance().m_rgtpivpKeyEvents.push_back(std::make_tuple(Qt::Key_Up, [=](){ geometry_msgs::Twist command;
                                                                                         command.linear.x = Globals::getInstance().m_dLinearAcceleration;
                                                                                         m_fcController.publishCommand(command);}));
    Globals::getInstance().m_rgtpivpKeyEvents.push_back(std::make_tuple(Qt::Key_Down, [=](){ geometry_msgs::Twist command;
                                                                                         command.linear.x = -Globals::getInstance().m_dLinearAcceleration;
                                                                                         m_fcController.publishCommand(command);}));
    Globals::getInstance().m_rgtpivpKeyEvents.push_back(std::make_tuple(Qt::Key_Left, [=](){ geometry_msgs::Twist command;
                                                                                         command.linear.y = Globals::getInstance().m_dLinearAcceleration;
                                                                                         m_fcController.publishCommand(command);}));
    Globals::getInstance().m_rgtpivpKeyEvents.push_back(std::make_tuple(Qt::Key_Right, [=](){ geometry_msgs::Twist command;
                                                                                         command.linear.y = -Globals::getInstance().m_dLinearAcceleration;
                                                                                         m_fcController.publishCommand(command);}));
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));
    readSettings();
    setWindowIcon(QIcon(":/images/icon.png"));
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }
}

MainWindow::~MainWindow()
{
    m_fcController.stopThread();
}

void MainWindow::keyPressEvent(QKeyEvent *p_pqkeEvent)
{
    for(size_t stIndex = 0; stIndex < Globals::getInstance().m_rgtpivpKeyEvents.size(); stIndex++)
    {
        if(std::get<0>(Globals::getInstance().m_rgtpivpKeyEvents.at(stIndex)) == p_pqkeEvent->key())
        {
            std::get<1>(Globals::getInstance().m_rgtpivpKeyEvents.at(stIndex))();

        }
    }
}

void MainWindow::showNoMasterMessage() {
    QMessageBox msgBox;
    msgBox.setText("Couldn't find the ros master.");
    msgBox.exec();
    close();
}

void MainWindow::takeOff()
{
    m_fcController.takeOff();
}

void MainWindow::on_button_connect_clicked(bool check ) {
    if(m_fcController.isConnected())
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
    QMainWindow::closeEvent(event);
}

void MainWindow::on_pushButton_clicked()
{
    new ImageConverter(0, NULL, this);
}
