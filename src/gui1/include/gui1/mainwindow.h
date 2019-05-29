#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "vizlib.h"
#include "mycp.h"
#include "leftwidget.h"
#include <rosnode.h>
#include <memory>
#include <QTimer>
#include <QProcess>
#include <ros/ros.h>
#include "customwidget.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    RosNode *rosNode;

public slots:
    void startMappingSlot();
    void customPath();
    void setStartPoint();
    void setEndPoint();
    void upload();
    void startNavigatingSlot();
    void saveLayout();
    void checkRosSlot();
    void closeProcessWidget();
protected:
    void closeEvent(QCloseEvent *event) override;
private:
    void loadLayout();
    QToolBar *toolBar;
    QAction *startMappingAction;
    QAction *customPathAction;
    QAction *startNavigationAction;
    VizlibTest *rvizWidget;
    MyCP *myCP;
    LeftWidget *leftWidget;
    QWidget *nullWidget;
    CustomDockWindow *helpWidget;
    QTimer *checkRosTimer;
    QProcess *keyControlProcess;
    QProcess *traceProcess;
    QString mapName;
    QSplitter *splitter;
    int status;// 0 无  1 建图  2 画路径 3完成绘制 4导航
    QProcess *commondProcess;
    ProcessWidget *processWidget;
    QLabel *statusBarLabel;
    void setInfomationToServer();
signals:
    void killkey();
};

#endif // MAINWINDOW_H
