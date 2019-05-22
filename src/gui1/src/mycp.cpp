#include "mycp.h"
#include <math.h>
#include "nlohmann/json.hpp"
#include <fstream>
#include <string>
#include "../include/gui1/moc_mycp.cpp"
#include "yaml-cpp/yaml.h"

using json = nlohmann::json;

MyCP::MyCP(QString mapName, QWidget *parent):m_mapName(mapName), QCustomPlot(parent)
{
    this->xAxis->setVisible(false);
    this->yAxis->setVisible(false);
    this->setBackground(QBrush(QColor(0xcd, 0xcd, 0xcd)));
    this->setMouseTracking(true);

    QString yamlFileQString = "/home/roboway/workspace/catkin_roboway/src/bringup/map/" + m_mapName + ".yaml";
    std::string yamlFileString = yamlFileQString.toStdString();
    YAML::Node config = YAML::LoadFile(yamlFileString);
    auto resolution = config["resolution"].as<double>();
    auto origin = config["origin"].as<std::vector<double>>();

    QString mapfilePath = "/home/roboway/workspace/catkin_roboway/src/bringup/map/" + m_mapName + "_modify.pgm";
    map = new QPixmap(mapfilePath, 0, Qt::MonoOnly);

    originToBottomLeftInMap = QPointF(origin[0] / resolution * -1, origin[1] / resolution * -1);//此程序标识符中的origin为建图的起点, yaml文件中的origin为左下角的点在map坐标系的坐标
    mapSize = map->size();

    qcpItem_map = new QCPItemPixmap(this);
    qcpItem_map->topLeft->setType(QCPItemPosition::ptPlotCoords);
    qcpItem_map->bottomRight->setType(QCPItemPosition::ptPlotCoords);
    qcpItem_map->setScaled(true, Qt::KeepAspectRatio);
    qcpItem_map->setPixmap(*map);
    qcpItem_map->setSelectable(false);

    top = mapSize.height() - originToBottomLeftInMap.y();
    left = 0 - originToBottomLeftInMap.x();
    bottom = 0 - originToBottomLeftInMap.y();
    right = mapSize.width() - originToBottomLeftInMap.x();

    qcpItem_map->topLeft->setCoords(left, top);
    qcpItem_map->bottomRight->setCoords(right, bottom);
    scale = 1;
    originMove = QSizeF(0, 0);

    originPoint = new QCPItemLine(this);
    originPoint->start->setCoords(0, 0);
    originPoint->end->setCoords(0.5,0);
    originPoint->setHead(QCPLineEnding::esDisc);
    originPoint->setTail(QCPLineEnding::esDisc);
    originPoint->setSelectable(false);
    QPen pen;
    pen.setColor(Qt::red);
    pen.setWidth(2);
    originPoint->setPen(pen);
    this->addItem(originPoint);

    this->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables | QCP::iSelectItems);
    drawTrace();
    this->replot();
}
void MyCP::createPathFile()
{
    json json_obj;
    json_obj["number"] = pathVector.size();
    for(int i = 0; i < pathVector.size(); i++)
    {
        const Pose &pose = pathVector[i];
        json_obj["pose"][i]["x"] = pose.point.x() / 10;
        json_obj["pose"][i]["y"] = pose.point.y() / 10;
        json_obj["pose"][i]["yaw"] = pose.yaw;
        json_obj["pose"][i]["type"] = pose.type;
    }

    std::ofstream out;
    QString planPathString = "/home/roboway/workspace/catkin_roboway/src/bringup/map/" + m_mapName + "_path.json";
    std::string planPath = planPathString.toStdString();
    out.open(planPath.c_str());

    out << json_obj.dump(4);
}
void MyCP::drawTrace()
{
    QString filename = "/home/roboway/workspace/catkin_roboway/src/bringup/map/" + m_mapName + "_trace.dat";
    QFile file(filename);
    if(file.open(QIODevice::ReadOnly))
    {
        QFileInfo info(file);
        qint64 size = info.size();
        QDataStream in(&file);

        QVector<double> keyVector;
        QVector<double> valueVector;

        for(qint64 i = 0; i < size - 1; i+=8)
        {
            double value;
            in.readRawData(reinterpret_cast<char *>(&value), 8);
            keyVector.append(value * 10);

            in.readRawData(reinterpret_cast<char *>(&value), 8);
            valueVector.append(value * 10);
        }
        file.close();
        addGraph();
        graph()->addData(keyVector, valueVector);
        graph()->setLineStyle(QCPGraph::lsNone);
        graph()->setScatterStyle(QCPScatterStyle::ssDisc);
    }
}
void MyCP::setStartPoint()
{
    planningPath = false;

    if(pathVector.size() >= 2)
    {
        pathVector.last().line->end->setCoords(pathVector.first().line->start->coords());
        Pose pose;
        pose.point = pathVector.first().point;
        pose.type = 0;
        pathVector.append(pose);

        for(int i = 1; i < pathVector.size(); i++)
        {
            pathVector[i].yaw = atan2((pathVector[i].point.y() - pathVector[i-1].point.y()) , (pathVector[i].point.x() - pathVector[i-1].point.x()));
        }
        pathVector[0].yaw = pathVector[1].yaw;
        pathVector.last().yaw = pathVector[0].yaw;//最后一个点是原点，yaw需要保持一致
        for(auto i : pathVector)
            qDebug() << i.point / 10 << "   " << i.yaw <<  "   " << i.type;
        createPathFile();
    }
    else {
        for(auto i : pathVector)
        {
            if(i.line)
                this->removeItem(i.line);
        }
        pathVector.clear();
    }
    this->replot();
}
void MyCP::setEndPoint()
{
    setEndFlag = true;
}
void MyCP::paintPath()
{
    if(!planningPath)
    {
        for(auto i : pathVector)
        {
            if(i.line)
                this->removeItem(i.line);
        }
        pathVector.clear();
        planningPath = true;
    }
    else
    {
        if(pathVector.size() >= 2)
        {
            this->removeItem(pathVector.last().line);
            pathVector.last().line = nullptr;
            pathVector.last().type = 1;


            //增加折返路径
            int points = pathVector.size();
            for(int i = points - 2; i >= 0; i--)
            {
                qDebug() << i << "  ";
                pathVector.append(pathVector[i]);
                pathVector.last().line = nullptr;
            }
            pathVector.last().type = 0;


            for(int i = 1; i < pathVector.size(); i++)
            {
                pathVector[i].yaw = atan2((pathVector[i].point.y() - pathVector[i-1].point.y()) , (pathVector[i].point.x() - pathVector[i-1].point.x()));
            }
            pathVector[0].yaw = pathVector[1].yaw;
            pathVector.last().yaw = pathVector[0].yaw;//最后一个点是原点，yaw需要保持一致

            for(auto i : pathVector)
                qDebug() << i.point / 10 << "   " << i.yaw <<  "   " << i.type;
            createPathFile();
        }
        else {
            for(auto i : pathVector)
            {
                if(i.line)
                {
                    this->removeItem(i.line);
                }
            }
            pathVector.clear();
        }
        planningPath = false;
    }
    this->replot();
}
void MyCP::wheelEvent(QWheelEvent*event)
{
    int numDegrees = event->angleDelta().y();
    double scale_mouse = (numDegrees < 0) ? 0.8 : 1.25;
    if(scale > 8 && scale_mouse > 1)
        return;
    if(scale < 1 && scale_mouse < 1)
        return;
    this->xAxis->setRange(this->xAxis->range() / scale_mouse);
    this->yAxis->setRange(this->yAxis->range() / scale_mouse);
    scale *= scale_mouse;

    //更新原点移动距离
    originMove.setWidth(this->xAxis->pixelToCoord(this->axisRect()->left() + this->axisRect()->width() / 2));
    originMove.setHeight(this->yAxis->pixelToCoord(this->axisRect()->top() + this->axisRect()->height() / 2));

    this->replot();
}


void MyCP::mousePressEvent(QMouseEvent *event)
{
    qDebug() << QPointF(this->xAxis->pixelToCoord(event->pos().x()), this->yAxis->pixelToCoord(event->pos().y()));
    if(planningPath == true)
    {
        pathPoint = event->pos();
    }

    QCustomPlot::mousePressEvent(event);
}

void MyCP::mouseMoveEvent(QMouseEvent *event)
{
    //更新原点移动距离
    originMove.setWidth(this->xAxis->pixelToCoord(this->axisRect()->left() + this->axisRect()->width() / 2));
    originMove.setHeight(this->yAxis->pixelToCoord(this->axisRect()->top() + this->axisRect()->height() / 2));

    if(planningPath && (pathVector.size() != 0))
    {
        QCPItemLine *line = pathVector.last().line;
        line->end->setCoords(xAxis->pixelToCoord(event->pos().x()), yAxis->pixelToCoord(event->pos().y()));
        line->setSelected(false);
    }

    this->replot();
    QCustomPlot::mouseMoveEvent(event);
}

void MyCP::mouseReleaseEvent(QMouseEvent *event)
{
    if(planningPath)
    {
        if(pathPoint == event->pos())
        {
            Pose pose;
            pose.point = QPointF(xAxis->pixelToCoord(event->pos().x()), yAxis->pixelToCoord(event->pos().y()));

            QCPItemLine *line = new QCPItemLine(this);
            line->start->setCoords(pose.point);
            line->end->setCoords(pose.point);
            line->setSelectable(true);

            QPen pen;
            pen.setColor(Qt::red);
            pen.setWidth(2);
            line->setPen(pen);
            line->setHead(QCPLineEnding::esLineArrow);

            pen.setColor(Qt::blue);
            line->setSelectedPen(pen);

            pose.line = line;
            if(setEndFlag == true)
            {
                pose.type = 1;
                setEndFlag = false;
            }
            if(pathVector.size() == 0)
                pose.type = 0;
            pathVector.append(pose);

            this->addItem(line);
        }
    }
    QCustomPlot::mouseReleaseEvent(event);
}


void MyCP::resizeEvent(QResizeEvent *event)
{
    QCustomPlot::resizeEvent(event);

    double width_height_ratio_map = mapSize.width() / mapSize.height();
    double width_height_ratio_widget = this->axisRect()->width() * 1.0 / this->axisRect()->height();

    if(width_height_ratio_map < width_height_ratio_widget) //地图窄-->高度对齐
    {
        this->yAxis->setRange(bottom / scale + originMove.height(), top / scale + originMove.height());
        this->xAxis->setRange(0 - width_height_ratio_widget * mapSize.height() / scale / 2 + originMove.width(), width_height_ratio_widget * mapSize.height() / scale / 2 + originMove.width());
    }
    else
    {                                                 //地图长-->水平对齐
        this->xAxis->setRange(left / scale + originMove.width(), right / scale + originMove.width());
        this->yAxis->setRange(0 - mapSize.width() / width_height_ratio_widget / scale / 2 + originMove.height(), mapSize.width() / width_height_ratio_widget / scale / 2 + originMove.height());
    }
    this->replot();
}
MyCP::~MyCP()
{
    if(map)
        delete map;
    for(auto i : pathVector)
    {
        if(i.line)
            this->removeItem(i.line);
    }
    if(originPoint)
        this->removeItem(originPoint);
}
