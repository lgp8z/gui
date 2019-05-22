#include "mainwindow.h"
#include <QApplication>
#include <rosnode.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "roboway_gui_node");
    ros::Time::init();
    QApplication a(argc, argv);
    RosNode rosNode;

    MainWindow w;
    w.rosNode = &rosNode;
    w.show();
    return a.exec();
}
