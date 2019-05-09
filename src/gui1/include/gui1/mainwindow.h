#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "vizlib.h"
#include "mycp.h"
#include "leftwidget.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
public slots:
    void startMappingSlot();
    void startNavigatingSlot();
    void saveLayout();
protected:
    void closeEvent(QCloseEvent *event) override;
private:
    void loadLayout();
    VizlibTest *rvizWidget;
    MyCP *myCP;
    LeftWidget *leftWidget;
    QWidget *nullWidget;
};

#endif // MAINWINDOW_H
