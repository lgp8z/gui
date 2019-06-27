#ifndef ROSNODE_H
#define ROSNODE_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <thread>
#include <memory>
#include <QProcess>

class RosNode
{
public:
    RosNode();
    ~RosNode();
    int getCarId(std::string &carIdString);
    void clearOdom();
    void startSlam();
    void stopSlam();
    void startNavigation(std::string mapName);
    void stopNavigation();
private:
    std::shared_ptr<std::thread> thread_ptr;
    ros::NodeHandle node;
    ros::ServiceClient motor_control_client;
    ros::ServiceClient agent_client;
    void gps_callback(const geometry_msgs::Pose::ConstPtr& msg);
    void func();
    double lantitude;
    double longitude;
};

#endif // ROSNODE_H
