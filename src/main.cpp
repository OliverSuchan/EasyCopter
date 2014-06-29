#include <QtGui>
#include <QApplication>
#include "../include/EasyCopter/main_window.hpp"

int main(int argc, char **argv) {
    QApplication app(argc, argv);
    MainWindow w;
    w.show();
    return app.exec();
}
