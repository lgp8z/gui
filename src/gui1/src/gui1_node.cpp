#include "mainwindow.h"
#include <QApplication>
#include <rosnode.h>
#include <QElapsedTimer>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "roboway_gui_node");
    ros::Time::init();
    QApplication a(argc, argv);
    RosNode rosNode;

    MainWindow w(&rosNode);
    w.show();
    return a.exec();
}
