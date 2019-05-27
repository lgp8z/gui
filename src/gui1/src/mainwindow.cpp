#include "mainwindow.h"
#include <QDebug>
#include <QTextEdit>
#include <QToolBar>
#include <QFile>
#include <QCloseEvent>
#include <QMessageBox>
#include "../include/gui1/moc_mainwindow.cpp"

#include <QtNetwork/QNetworkRequest>
#include <QtNetwork/QNetworkReply>
#include <QtNetwork/QNetworkAccessManager>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent)
{
    setObjectName("MainWindow");
    setWindowTitle(QStringLiteral("控制界面"));
    setWindowState(Qt::WindowMaximized);

    status = 0;
    myCP = nullptr;
    processWidget = new ProcessWidget(this);
    processWidget->setModal(true);
    processWidget->hide();

    rvizWidget = new VizlibTest(this);
    leftWidget = new LeftWidget(this);
    leftWidget->hide();

    splitter = new QSplitter(this);
    splitter->addWidget(leftWidget);
    splitter->addWidget(rvizWidget);
    splitter->setStretchFactor(0,0);
    splitter->setStretchFactor(1,1);
    for(int i = 0; i < splitter->count(); i++)
    {
        splitter->handle(i)->setEnabled(false);
    }

    nullWidget = new QWidget(this);
    setCentralWidget(nullWidget);

    CustomDockWindow *helpWidget = new CustomDockWindow(QStringLiteral("帮助"), this);
    helpWidget->setObjectName("helpWidget");
    addDockWidget(Qt::RightDockWidgetArea, helpWidget);

    QToolBar *toolBar = new QToolBar(this);
    toolBar->setObjectName("toobar");

    startMappingAction = new QAction(QStringLiteral("开始建图"));
    startMappingAction->setCheckable(true);
    connect(startMappingAction, SIGNAL(triggered(bool)), this, SLOT(startMappingSlot()));
    toolBar->addAction(startMappingAction);
    toolBar->addSeparator();

    customPathAction = new QAction(QStringLiteral("绘制路径"));
    customPathAction->setCheckable(true);
    connect(customPathAction, SIGNAL(triggered(bool)), this, SLOT(customPath()));
    toolBar->addAction(customPathAction);
    toolBar->addAction(QStringLiteral("设置终点"), this, SLOT(setEndPoint()));
    toolBar->addAction(QStringLiteral("设置起点"), this, SLOT(setStartPoint()));
    toolBar->addSeparator();

    toolBar->addAction(QStringLiteral("上传地图"), this, SLOT(upload()));
    toolBar->addSeparator();

    startNavigationAction = new QAction(QStringLiteral("启动业务"));
    startNavigationAction->setCheckable(true);
    connect(startNavigationAction, SIGNAL(triggered(bool)), this, SLOT(startNavigatingSlot()));
    toolBar->addAction(startNavigationAction);
    toolBar->addSeparator();

    toolBar->addAction(QStringLiteral("帮助"), this, [=](){helpWidget->show();});
    addToolBar(toolBar);

    statusBarLabel = new QLabel(this);
    statusBar()->addWidget(statusBarLabel);

    keyControlProcess = new QProcess();
    traceProcess = new QProcess();
    commondProcess = new QProcess();

    checkRosTimer = new QTimer(this);
    checkRosTimer->setInterval(1000);
    checkRosTimer->start();
    connect(checkRosTimer, SIGNAL(timeout()), this, SLOT(checkRosSlot()));

    //loadLayout();
}

void MainWindow::checkRosSlot()
{
    if(ros::master::check())
    {
        if(QProcess::NotRunning == keyControlProcess->state())
        {
            keyControlProcess->start("roslaunch bringup key_control.launch");
        }
    }
    else
    {
        if(QProcess::NotRunning != keyControlProcess->state())
        {
            keyControlProcess->terminate();
            keyControlProcess->waitForFinished();
        }
    }
}
void MainWindow::saveLayout()
{
    QFile file("layout.cfg");
    file.open(QFile::WriteOnly);

    QByteArray geo_data = saveGeometry();
    QByteArray layout_data = saveState();

    file.putChar((uchar)geo_data.size());
    file.write(geo_data);
    file.write(layout_data);
}

void MainWindow::loadLayout()
{
    QFile file("layout.cfg");
    if(!file.open(QFile::ReadOnly))
        return;

    uchar geo_size;
    QByteArray geo_data;
    QByteArray layout_data;

    file.getChar((char*)&geo_size);
    geo_data = file.read(geo_size);
    layout_data = file.readAll();

    restoreGeometry(geo_data);
    restoreState(layout_data);
}

void MainWindow::setInfomationToServer()
{
    QNetworkAccessManager* manager = new QNetworkAccessManager;

    QString postString =
                  QString("number=2")
                + QString("&communicationId=000002")
                + QString("&startSiteName=") + leftWidget->getStartName()
                + QString("&startLongitude=") + QString::number(120)
                + QString("&startLatitude=") + QString::number(33)
                + QString("&endSiteName=") + leftWidget->getEndName()
                + QString("&endLongitude=") + QString::number(120.1)
                + QString("&endLatitude=") + QString::number(33.1)
                + QString("&token=00");
    qDebug() << postString;

    QByteArray post =  postString.toUtf8();

    QNetworkRequest req;
    QString qurl = "http://api.roboway.cn/vehicleWeb/user/vehicle/saveSiteAndCommand?";
    req.setUrl(QUrl(qurl));
    req.setHeader(QNetworkRequest::ContentTypeHeader, "application/x-www-form-urlencoded");
    manager->post(req, post);
    QObject::connect(manager, &QNetworkAccessManager::finished, [](QNetworkReply* reply){
            if(reply->error() != QNetworkReply::NoError)
            {
                qDebug() << "Error:" << reply->errorString();
                return;
            }
            QByteArray buf = reply->readAll();
            qDebug() << "OK:"<< buf;
    });
}

void MainWindow::startMappingSlot()
{
    QAction *action = (QAction *)sender();

    if(status == 0 || status == 3)
    {
        if(!ros::master::check()) {
            QMessageBox::warning(this, QStringLiteral("注意"), QStringLiteral("未连接机器人Wifi,不能执行此操作!"));
            startMappingAction->setChecked(false);
            return;
        }
        QLineEditDialog dialog(QString(), this);
        if(dialog.exec() == QDialog::Rejected)
        {
            startMappingAction->setChecked(false);
            return;
        }
        mapName = dialog.enteredString();

        rosNode->clearOdom();
        rosNode->startSlam();
        traceProcess->start("rosrun roboway_tool trace " + mapName);

        action->setText(QStringLiteral("停止建图"));
        centralWidget()->setParent(0);
        setCentralWidget(splitter);
        leftWidget->show();
        rvizWidget->display_Slam();
        status = 1;
    }
    else if(status == 1)
    {
        if(leftWidget->getStartName().isEmpty() || leftWidget->getEndName().isEmpty())
        {
            QMessageBox::warning(this, QStringLiteral("注意"), QStringLiteral("未输入有效的起止地点!"));
            startMappingAction->setChecked(true);
            return;
        }

        setInfomationToServer();

        QProcess process;
        process.start("/home/roboway/workspace/catkin_roboway/src/bringup/script/save_map.sh " + mapName);
        process.waitForFinished();
        traceProcess->terminate();
        traceProcess->waitForFinished();
        rosNode->stopSlam();

        QMessageBox::information(this, QStringLiteral("提示"), QStringLiteral("已停止建图, 请绘制路径"));
        centralWidget()->setParent(0);
        leftWidget->hide();
        rvizWidget->quit();
        setCentralWidget(nullWidget);
        action->setText(QStringLiteral("开始建图"));
        status = 0;
    }
    else
    {
        QMessageBox::warning(this, QStringLiteral("注意"), QStringLiteral("请先终止其他操作!"));
        startMappingAction->setChecked(false);
    }
}

void MainWindow::customPath()
{
    qDebug() << "ddd";
    if(status == 0 || status == 3) //开始画路径
    {
        QLineEditDialog dialog(mapName, this);
        if(dialog.exec() == QDialog::Rejected)
        {
            customPathAction->setChecked(false);
            return;
        }
        QString mapName_input = dialog.enteredString();

        status = 2;
        if(myCP != nullptr)//重画路径时如果是同一张地图,不需要new
        {
            if(myCP->m_mapName != mapName_input)
            {
                delete myCP;
                myCP = new MyCP(mapName_input, this);
            }
        }
        else
        {
            myCP = new MyCP(mapName_input, this);
        }

        centralWidget()->setParent(0);
        setCentralWidget(myCP);
        myCP->paintPath();
    }
    else if(status == 2)//绘制完成
    {
        status = 3;
        myCP->paintPath();
    }
    else
    {
        QMessageBox::warning(this, QStringLiteral("注意"), QStringLiteral("请先终止其他操作!"));
        customPathAction->setChecked(false);
    }
}
void MainWindow::setStartPoint()
{
    if(status == 2)
    {
        myCP->setStartPoint();
        status = 3;
        customPathAction->setChecked(false);//同时需要把绘制路径的action的图标设置为un check
    }
}
void MainWindow::setEndPoint()
{
    if(status == 2)
        myCP->setEndPoint();
}
void MainWindow::upload()
{
    QLineEditDialog dialog(mapName, this);
    if(dialog.exec() == QDialog::Rejected)
        return;
    QString mapName_input = dialog.enteredString();

    processWidget->show();
    connect(commondProcess, SIGNAL(finished(int)), SLOT(closeProcessWidget()));
    commondProcess->start("/home/roboway/workspace/catkin_roboway/src/bringup/script/sync_file.sh " + mapName_input);
}
void MainWindow::closeProcessWidget()
{
    processWidget->close();
}
void MainWindow::startNavigatingSlot()
{
    QAction *action = (QAction *)sender();

    if(status == 0 || status == 3)
    {
        if(0 == rosNode->getAgentStatus())//打开软件后 根据车子情况自动进入导航状态
        {
            rosNode->clearOdom();
            rosNode->startNavigation();
        }

        centralWidget()->setParent(0);
        setCentralWidget(splitter);
        leftWidget->hide();
        rvizWidget->display_Navigation();
        action->setText(QStringLiteral("停止业务"));
        status = 4;
    }
    else if(status == 4)
    {
        rosNode->stopNavigation();
        centralWidget()->setParent(0);
        leftWidget->hide();
        rvizWidget->quit();
        setCentralWidget(nullWidget);
        action->setText(QStringLiteral("启动业务"));
        status = 0;
    }
    else
    {
        QMessageBox::warning(this, QStringLiteral("注意"), QStringLiteral("请先终止其他操作!"));
        startNavigationAction->setChecked(false);
    }
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    saveLayout();
    event->accept();
}
MainWindow::~MainWindow()
{
    keyControlProcess->terminate();
    keyControlProcess->waitForFinished();
    delete keyControlProcess;
    delete traceProcess;

    if(myCP)
        delete myCP;
    if(rvizWidget)
        delete rvizWidget;
}
