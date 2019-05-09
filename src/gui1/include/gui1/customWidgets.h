#ifndef CUSTOMDOCKWINDOW_H
#define CUSTOMDOCKWINDOW_H

#include <QObject>
#include <QDockWidget>
#include <QScrollArea>
#include <QCloseEvent>
#include <QLineEdit>
#include <QDialog>
#include <QGridLayout>
#include <QDialogButtonBox>
#include <QLabel>

class QLineEditDialog : public QDialog
{
public:
    explicit QLineEditDialog(QWidget *parent = nullptr);
    QString enteredString() const { return m_objectName->text(); }
private:
    QLineEdit *m_objectName;
};
QLineEditDialog::QLineEditDialog(QWidget *parent): QDialog(parent), m_objectName(new QLineEdit(this))
{
    setWindowTitle(QStringLiteral("请输入地图名称"));
    setWindowFlags(windowFlags() & ~Qt::WindowContextHelpButtonHint);
    QGridLayout *layout = new QGridLayout(this);

    layout->addWidget(new QLabel(QStringLiteral("地图名称：")), 0, 0);
    layout->addWidget(m_objectName, 0, 1);

    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
    layout->addWidget(buttonBox, 1, 0, 1, 2);
}

class CustomDockWindow : public QDockWidget
{
public:
    explicit CustomDockWindow(const QString &title, QWidget *parent = nullptr): QDockWidget(title, parent){
        setFeatures(features() & ~QDockWidget::DockWidgetMovable);
        QScrollArea * helpScrollArea = new QScrollArea(this);
        QWidget *widget = new QWidget(helpScrollArea);
        widget->setStyleSheet("QWidget{background-color: rgba(0,255,255,100)}");
        helpScrollArea->setViewport(widget);
        setWidget(helpScrollArea);
    }
};

#endif // CUSTOMDOCKWINDOW_H
