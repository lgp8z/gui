#include "rosnode.h"
#include "agent/commond.h"
#include "motor_control/motor_commond.h"

#include <QDebug>
RosNode::RosNode()
{
    motor_control_client = node.serviceClient<motor_control::motor_commond>("motor_control/commond");
    agent_client = node.serviceClient<agent::commond>("agent/commond");

    thread_ptr = std::make_shared<std::thread>(&RosNode::func, this);
    thread_ptr->detach();
}
void RosNode::func()
{
    ros::Subscriber gpsSub = node.subscribe<geometry_msgs::Pose>("/gps", 10, &RosNode::gps_callback, this);//处理joy
    ros::spin();
}

void RosNode::gps_callback(const geometry_msgs::Pose::ConstPtr& msg)
{
    lantitude = msg->position.x;
    longitude = msg->position.y;
    ROS_INFO_STREAM("lantitude: " << lantitude << "longitude: " << longitude);
}
void RosNode::clearOdom()
{
    motor_control::motor_commond motor_commond_srv;
    motor_commond_srv.request.commond = 3;
    motor_control_client.call(motor_commond_srv);
}

void RosNode::startSlam()
{
    agent::commond agent_commond_srv;
    agent_commond_srv.request.type = 0;
    agent_client.call(agent_commond_srv);
}
void RosNode::stopSlam()
{
    agent::commond agent_commond_srv;
    agent_commond_srv.request.type = 1;
    agent_client.call(agent_commond_srv);
}
void RosNode::startNavigation()
{
    agent::commond agent_commond_srv;
    agent_commond_srv.request.type = 2;
    agent_client.call(agent_commond_srv);
}
void RosNode::stopNavigation()
{
    agent::commond agent_commond_srv;
    agent_commond_srv.request.type = 3;
    agent_client.call(agent_commond_srv);
}
int RosNode::getAgentStatus()
{
    agent::commond agent_commond_srv;
    agent_commond_srv.request.type = 4;
    agent_client.call(agent_commond_srv);
    return agent_commond_srv.response.returntype;
}
RosNode::~RosNode()
{
}
