#ifndef MYCP_H
#define MYCP_H

#include <QWidget>
#include "qcustomplot.h"
#include <QPixmap>

struct Pose
{
    Pose():point(0, 0), yaw(0), type(-1), line(nullptr){}
    QPointF point;
    double yaw;
    int type;//0:起点 1:终点 -1:普通
    QCPItemLine *line;
};

class MyCP : public QCustomPlot
{
    Q_OBJECT
public:
    MyCP(QWidget *parent = nullptr);
    ~MyCP();
public slots:
    void paintPath();
    void setStartPoint();
    void setEndPoint();
protected:
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void resizeEvent(QResizeEvent *event);
    void wheelEvent(QWheelEvent*event);
private:
    QPixmap *map{nullptr};
    QSizeF mapSize;
    double ratio_map;
    QPointF originToBottomLeftInMap;
    int top, bottom, left, right;
    QCPItemPixmap *qcpItem_map{nullptr};
    double scale;
    bool paintPathFlag{false};
    QPoint pathPoint;
    QSizeF originMove;
    QVector<Pose> pathVector;
    bool setEndFlag{false};
    bool planningPath{false};
    void createPathFile();
    QCPItemLine *originPoint;
    QString mapName;
    void drawTrace();
};

#endif // MYCP_H
