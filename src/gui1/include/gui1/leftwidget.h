#ifndef LEFTWIDGET_H
#define LEFTWIDGET_H

#include <QWidget>

namespace Ui {
class LeftWidget;
}

class LeftWidget : public QWidget
{
    Q_OBJECT

public:
    explicit LeftWidget(QWidget *parent = 0);
    ~LeftWidget();
    QString getStartName();
    QString getEndName();
    void setStartLantitude(QString str);
    void setStartLongitude(QString str);
    void setEndLantitude(QString str);
    void setEndLongitude(QString str);
    void setCurrentLantitude(QString str);
    void setCurrentLongitude(QString str);
private:
    Ui::LeftWidget *ui;
};

#endif // LEFTWIDGET_H
