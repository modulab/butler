#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <navfn/navfn_ros.h>

#include <math.h>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/String.h"
//using namespace geometry_msgs;

// %Tag(vars)%
ros::Publisher validGoalPub, invalidGoalPub, validPlanPub;
navfn::NavfnROS navfna, navfn_no_sensor;
// %EndTag(vars)%

// %Tag(frameCallback)%
void frameCallback(const ros::TimerEvent&)
{
  tf::Transform t;

  ros::Time time = ros::Time::now();
}
// %EndTag(frameCallback)%

// %Tag(goalCallback)%
void goalCallback(const geometry_msgs::PoseArray::ConstPtr& goal)
{
geometry_msgs::Pose a,b;
geometry_msgs::PoseStamped a2,b2;
std::vector<geometry_msgs::PoseStamped> c;

a2.pose=goal->poses[0];
b2.pose=goal->poses[1];
a2.header.frame_id="map";
b2.header.frame_id="map";


bool succeeded = navfna.makePlan(a2,b2,c);
// bool succeeded;
ROS_INFO(succeeded ? "True" : "False");


std_msgs::String msg;
msg.data = goal->header.frame_id;

if(succeeded){
validGoalPub.publish(msg);
} else {
invalidGoalPub.publish(msg);
}

}
// %EndTag(goalCallback)%

// %Tag(planCallback)%

void planCallback(const geometry_msgs::PoseArray::ConstPtr& goal)
{
geometry_msgs::Pose a,b;
geometry_msgs::PoseStamped a2,b2;
std::vector<geometry_msgs::PoseStamped> c;


a2.pose=goal->poses[0];
b2.pose=goal->poses[1];
a2.header.frame_id="map";
b2.header.frame_id="map";


bool succeeded = navfn_no_sensor.makePlan(a2,b2,c);

if(succeeded){
std_msgs::String msg;
msg.data = goal->header.frame_id;
//validPlanPub.publish(msg);
ROS_INFO("Sending initial plan");
}
else {
ROS_INFO("Failed to find initial plan");
}

}

// %EndTag(planCallback)%


// %Tag(main)%
int main(int argc, char** argv)
{
  ros::init(argc, argv, "butler_planner");
  ros::NodeHandle n;

  // create a timer to update the published transforms
  ros::Timer frame_timer = n.createTimer(ros::Duration(0.01), frameCallback);

  ros::Duration(1.5).sleep();


tf::TransformListener tfa(ros::Duration(15));
costmap_2d::Costmap2DROS costmap("global_costmap", tfa);
navfna.initialize("planner", &costmap);


costmap_2d::Costmap2DROS costmap_no_sensor("global_costmap_no_sensor", tfa);
navfn_no_sensor.initialize("planner_no_sensor", &costmap_no_sensor);


validGoalPub = n.advertise<std_msgs::String>("butler/planner/valid_goal", 1000);
ros::Subscriber goalSub = n.subscribe("butler/planner/goal", 1, goalCallback);

ros::Subscriber planSub = n.subscribe("butler/planner/make_plan", 1, planCallback);

invalidGoalPub = n.advertise<std_msgs::String>("butler/planner/invalid_goal", 1000);

  ros::spin();
}
// %EndTag(main)%
