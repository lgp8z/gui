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
#include <QProcess>
#include <QScopedPointer>
#include <functional>
#include <QPushButton>
#include <QMessageBox>
#include <QSpacerItem>
#include <QVBoxLayout>
#include <QDebug>


class CancelMessageBox : public QMessageBox
{
    Q_OBJECT

public:
    CancelMessageBox(QString text, QWidget *parent = nullptr);
public slots:
    void setCancel();
    void setOk();
protected:
    void closeEvent(QCloseEvent *event);
};

class ProcessWidget : public QDialog
{
public:
    explicit ProcessWidget(QWidget *parent = nullptr);
};

class QLineEditDialog : public QDialog
{
    Q_OBJECT

public:
    explicit QLineEditDialog(QString defaultString, QWidget *parent = nullptr);
    QString enteredString() const ;
public slots:
    virtual void accept();
private:
    QLineEdit *m_objectName;
};

class CustomDockWindow : public QDockWidget
{
public:
    explicit CustomDockWindow(const QString &title, QWidget *parent = nullptr);
};

#endif // CUSTOMDOCKWINDOW_H
