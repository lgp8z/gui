#include "mainwindow.h"
#include <QApplication>
#include <ros/ros.h>
#include <string>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "roboway_gui_node");
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}
