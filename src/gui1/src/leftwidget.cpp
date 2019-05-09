#include "leftwidget.h"
#include "ui_leftwidget.h"
#include "../include/gui1/moc_leftwidget.cpp"

LeftWidget::LeftWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::LeftWidget)
{
    ui->setupUi(this);
}

LeftWidget::~LeftWidget()
{
    delete ui;
}
