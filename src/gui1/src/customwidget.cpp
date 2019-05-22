#include "customwidget.h"
#include "../include/gui1/moc_customwidget.cpp"

ProcessWidget::ProcessWidget(QWidget *parent): QDialog(parent)
{
    this->setWindowTitle(QStringLiteral("注意"));
    this->setMaximumSize(10, 10);
    QVBoxLayout *layout = new QVBoxLayout(this);
    QLabel *label = new QLabel(QStringLiteral("正在处理相关操作, 请等待!"), this);
    layout->addWidget(label);
    QHBoxLayout *layout_ = new QHBoxLayout();
    QSpacerItem *horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);
    layout_->addItem(horizontalSpacer);
    QPushButton *button = new QPushButton(QStringLiteral("终止"), this);
    layout_->addWidget(button);
    QSpacerItem *horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);
    layout_->addItem(horizontalSpacer_2);
    layout->addLayout(layout_);
}

QString QLineEditDialog::enteredString() const
{
    return m_objectName->text();
}
QLineEditDialog::QLineEditDialog(QString defaultString, QWidget *parent): QDialog(parent), m_objectName(new QLineEdit(defaultString, this))
{
    setWindowTitle(QStringLiteral("请输入地图名称"));
    setWindowFlags(windowFlags() & ~Qt::WindowContextHelpButtonHint);
    QGridLayout *layout = new QGridLayout(this);
    QLabel *label = new QLabel(this);
    label->setText(QStringLiteral("   请以英文开头,不允许输入中文!"));
    layout->addWidget(new QLabel(QStringLiteral("地图名称：")), 0, 0);
    layout->addWidget(m_objectName, 0, 1);
    layout->addWidget(label, 1, 0, 1, 2);
    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
    layout->addWidget(buttonBox, 2, 0, 1, 2);
}
void QLineEditDialog::accept()
{
    if(m_objectName->text().isEmpty())
    {
        QMessageBox::warning(this, QStringLiteral("注意"), QStringLiteral("地图名不能为空!"));
        return;
    }
    QDialog::accept();
}

CustomDockWindow::CustomDockWindow(const QString &title, QWidget *parent): QDockWidget(title, parent){
    setFeatures(features() & ~QDockWidget::DockWidgetMovable);
    QScrollArea * helpScrollArea = new QScrollArea(this);
    QWidget *widget = new QWidget(helpScrollArea);
    widget->setStyleSheet("QWidget{background-color: rgba(0,255,255,100)}");
    helpScrollArea->setViewport(widget);
    setWidget(helpScrollArea);
}
