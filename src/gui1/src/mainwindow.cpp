#include "mainwindow.h"
#include <QDebug>
#include <QTextEdit>
#include <QToolBar>
#include <QFile>
#include <QCloseEvent>
#include <QMessageBox>
#include "../include/gui1/moc_mainwindow.cpp"
#include "customWidgets.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent)
{
    setObjectName("MainWindow");
    setWindowTitle(QStringLiteral("控制界面"));
    setWindowState(Qt::WindowMaximized);

    rvizWidget = new VizlibTest(this);
    myCP = new MyCP(this);
    leftWidget = new LeftWidget(this);
    leftWidget->hide();
    nullWidget = new QWidget(this);

    setCentralWidget(nullWidget);

    CustomDockWindow *helpWidget = new CustomDockWindow(QStringLiteral("帮助"), this);
    helpWidget->setObjectName("helpWidget");
    addDockWidget(Qt::RightDockWidgetArea, helpWidget);

    QToolBar *toolBar = new QToolBar(this);
    toolBar->setObjectName("toobar");
    toolBar->addAction(QStringLiteral("开始建图"), this, SLOT(startMappingSlot()));
    toolBar->addSeparator();
    toolBar->addAction(QStringLiteral("自定义路径"), this, [=](){centralWidget()->setParent(0);setCentralWidget(myCP); myCP->paintPath();});
    toolBar->addAction(QStringLiteral("设置终点"), myCP, SLOT(setStartPoint()));
    toolBar->addAction(QStringLiteral("设置起点"), myCP, SLOT(setEndPoint()));
    toolBar->addSeparator();
    toolBar->addAction(QStringLiteral("启动业务"), this, SLOT(startNavigatingSlot()));
    toolBar->addSeparator();
    toolBar->addAction(QStringLiteral("帮助"), this, [=](){helpWidget->show();});
    addToolBar(toolBar);

    //loadLayout();
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

void MainWindow::startMappingSlot()
{
    QAction *action = (QAction *)sender();
    QLineEditDialog dialog;
    if(action->text() == QStringLiteral("开始建图"))
    {
        if(dialog.exec() == QDialog::Rejected)
            return;
        action->setText(QStringLiteral("停止建图"));
        QString mapName = dialog.enteredString();
        centralWidget()->setParent(0);
        setCentralWidget(rvizWidget);
        rvizWidget->display();
    }
    else {
        action->setText(QStringLiteral("开始建图"));
    }
}
void MainWindow::startNavigatingSlot()
{
    QAction *action = (QAction *)sender();
    if(action->text() == QStringLiteral("启动业务"))
    {
        centralWidget()->setParent(0);
        setCentralWidget(rvizWidget);
        rvizWidget->display();
        action->setText(QStringLiteral("停止业务"));
    }
    else {
        action->setText(QStringLiteral("启动业务"));
    }
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    saveLayout();
    event->accept();
}
MainWindow::~MainWindow()
{
    delete myCP;
    delete rvizWidget;
}
