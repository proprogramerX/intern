#include <ros/ros.h>
#include "sensor_coverage_planner/sensor_coverage_planner_ground.h"

#include "../../include/grid_world/grid_world.h"

#include "std_msgs/String.h"

#include <sstream>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "node");
  ros::NodeHandle nh;
  ros::NodeHandle private_node_handle("~");

  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("msg", 1000);

  std::vector<int> vlist{};
  std::vector<int> list{};

  sensor_coverage_planner_3d_ns::PlannerData pd;
  //pd.grid_world_ = std::make_unique<grid_world_ns::GridWorld>(nh);

//  pd.grid_world_->GetExploringCellIndices(pd.explore_sub);
//  sensor_coverage_planner_3d_ns::SensorCoveragePlanner3D

  for (auto it = list.begin(); it != list.end(); ++it)
    std::cout << ' ' << *it;

  //gw.GetExploringCellIndices(vlist);

  //vlist = pd.grid_world_->r();

  std_msgs::String msg;

	for (auto it = vlist.begin(); it != vlist.end(); ++it)
    std::cout << ' ' << *it;

  std::stringstream ss;
  msg.data = ss.str();

  ROS_INFO("%s", msg.data.c_str());

  chatter_pub.publish(msg);

  ros::Rate loop_rate(10);

  ros::spin();
  return 0;
}