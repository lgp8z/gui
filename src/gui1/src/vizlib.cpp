#include <QVBoxLayout>
#include "vizlib.h"

VizlibTest::VizlibTest(QWidget *parent) :
    QWidget(parent)
{
    render_panel_ = new rviz::RenderPanel();
    manager_ = new rviz::VisualizationManager(render_panel_);
    QVBoxLayout *verticalLayout = new QVBoxLayout(this);
    verticalLayout->addWidget( render_panel_ );
}

VizlibTest::~VizlibTest()
{
    if(manager_ != nullptr)
    {
        manager_->removeAllDisplays();
    }
}

void VizlibTest::display_Slam()
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
void VizlibTest::display_Navigation()
{
    render_panel_->initialize( manager_->getSceneManager(), manager_ );
    manager_->initialize();

    rviz::Display *show_map = manager_->createDisplay( "rviz/Map", "adjustable map", true );
    show_map->subProp( "Topic" )->setValue( "/show_map" );

    rviz::Display *local_castmap = manager_->createDisplay( "rviz/Map", "adjustable local castmap", true );
    local_castmap->subProp( "Topic" )->setValue( "/move_base/local_costmap/costmap" );

//    rviz::Display *globalPlan = manager_->createDisplay( "rviz/Path", "global plan", true );
//    globalPlan->subProp( "Topic" )->setValue( "/move_base/TrajectoryPlannerROS/global_plan" );

    rviz::Display *robot = manager_->createDisplay( "rviz/RobotModel", "adjustable robot", true );
    robot->subProp( "Robot Description" )->setValue( "robot_description" );

    manager_->startUpdate();
}
void VizlibTest::quit()
{
    manager_->stopUpdate();
    if(manager_ != nullptr)
    {
        manager_->removeAllDisplays();
    }
}
