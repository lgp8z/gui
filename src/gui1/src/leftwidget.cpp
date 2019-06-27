#include "leftwidget.h"
#include "ui_leftwidget.h"

LeftWidget::LeftWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::LeftWidget)
{
    ui->setupUi(this);
}
QString LeftWidget::getStartName()
{
    return ui->startNameLineEdit->text();
}
QString LeftWidget::getEndName()
{
    return ui->endNameLineEdit->text();
}
void LeftWidget::setStartLantitude(QString str)
{
    ui->startLantitudeLineEdit->setText(str);
}
void LeftWidget::setStartLongitude(QString str)
{
    ui->startLongitudeLineEdit->setText(str);
}
void LeftWidget::setEndLantitude(QString str)
{
    ui->endLantitudeLineEdit->setText(str);
}
void LeftWidget::setEndLongitude(QString str)
{
    ui->endLongitudeLineEdit->setText(str);
}
void LeftWidget::setCurrentLantitude(QString str)
{
    ui->currentLantitudeLineEdit->setText(str);
}
void LeftWidget::setCurrentLongitude(QString str)
{
    ui->currentLongitudeLineEdit->setText(str);
}
LeftWidget::~LeftWidget()
{
    delete ui;
}
