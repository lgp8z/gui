#include <QVBoxLayout>
#include "vizlib.h"

VizlibTest::VizlibTest(QWidget *parent) :
    QWidget(parent)
{
    render_panel_ = new rviz::RenderPanel(this);
    manager_ = new rviz::VisualizationManager( render_panel_ );
    QVBoxLayout *verticalLayout = new QVBoxLayout(this);
    verticalLayout->addWidget( render_panel_ );
}

VizlibTest::~VizlibTest()
{
    manager_->stopUpdate();
    delete manager_;
}

void VizlibTest::display()
{
    render_panel_->initialize( manager_->getSceneManager(), manager_ );
    manager_->initialize();

    rviz::Display *map = manager_->createDisplay( "rviz/Map", "adjustable map", true );
    map->subProp( "Topic" )->setValue( "/map" );

    rviz::Display *robot = manager_->createDisplay( "rviz/RobotModel", "adjustable robot", true );
    robot->subProp( "Robot Description" )->setValue( "robot_description" );

//    rviz::Display *laser = manager_->createDisplay( "rviz/LaserScan", "adjustable scan", true );
//    laser->subProp( "Topic" )->setValue( "/scan" );
//    laser->subProp( "Size (m)" )->setValue( "0.1" );
    manager_->startUpdate();
}
