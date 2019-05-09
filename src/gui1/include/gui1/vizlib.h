#ifndef RVIZ_H
#define RVIZ_H

#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/tool_manager.h>
#include <QWidget>

class VizlibTest : public QWidget
{
public:
    explicit VizlibTest(QWidget *parent = 0);
    ~VizlibTest();

    void display();
    void quit();
private:
    rviz::VisualizationManager *manager_;
    rviz::RenderPanel * render_panel_;
};

#endif // RVIZ_H
