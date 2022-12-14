/*
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

const double PI = 3.1415926;

string metricFile;
string trajFile;
string mapFile;
double overallMapVoxelSize = 0.5;
double exploredAreaVoxelSize = 0.3;
double exploredVolumeVoxelSize = 0.5;
double transInterval = 0.2;
double yawInterval = 10.0;
int overallMapDisplayInterval = 2;
int overallMapDisplayCount = 0;
int exploredAreaDisplayInterval = 1;
int exploredAreaDisplayCount = 0;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZ>::Ptr overallMapCloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr overallMapCloudDwz(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZI>::Ptr exploredAreaCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr exploredAreaCloud2(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr exploredVolumeCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr exploredVolumeCloud2(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr trajectory(new pcl::PointCloud<pcl::PointXYZI>());

const int systemDelay = 5;
int systemDelayCount = 0;
bool systemDelayInited = false;
double systemTime = 0;
double systemInitTime = 0;
bool systemInited = false;

float vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
float exploredVolume = 0, travelingDis = 0, runtime = 0, timeDuration = 0;

pcl::VoxelGrid<pcl::PointXYZ> overallMapDwzFilter;
pcl::VoxelGrid<pcl::PointXYZI> exploredAreaDwzFilter;
pcl::VoxelGrid<pcl::PointXYZI> exploredVolumeDwzFilter;

sensor_msgs::PointCloud2 overallMap2;

FILE *metricFilePtr = NULL;
FILE *trajFilePtr = NULL;

pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_c(new pcl::PointCloud<pcl::PointXYZI>());

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudIn)
{
  if (!systemDelayInited)
  {
    systemDelayCount++;
    if (systemDelayCount > systemDelay)
    {
      systemDelayInited = true;
    }
  }

  if (!systemInited)
  {
    return;
  }

  laserCloud->clear();
  pcl::fromROSMsg(*laserCloudIn, *laserCloud);

  *exploredVolumeCloud += *laserCloud;

  exploredVolumeCloud2->clear();
  exploredVolumeDwzFilter.setInputCloud(exploredVolumeCloud);
  exploredVolumeDwzFilter.filter(*exploredVolumeCloud2);

  pcl::PointCloud<pcl::PointXYZI>::Ptr tempCloud = exploredVolumeCloud;
  exploredVolumeCloud = exploredVolumeCloud2;
  exploredVolumeCloud2 = tempCloud;

  *exploredAreaCloud += *laserCloud;

  cloud_c = *laserCloud;
  // cloud_c += exploredVolumeCloud;
  std::cerr << "Cloud C: " << std::endl;
  for (std::size_t i = 0; i < cloud_c.size(); ++i)
    std::cerr << "    " << cloud_c[i].x << " " << cloud_c[i].y << " " << cloud_c[i].z << " " << std::endl;

  exploredAreaDisplayCount++;
  if (exploredAreaDisplayCount >= 5 * exploredAreaDisplayInterval)
  {
    exploredAreaCloud2->clear();
    exploredAreaDwzFilter.setInputCloud(exploredAreaCloud);
    exploredAreaDwzFilter.filter(*exploredAreaCloud2);

    tempCloud = exploredAreaCloud;
    exploredAreaCloud = exploredAreaCloud2;
    exploredAreaCloud2 = tempCloud;

    exploredAreaDisplayCount = 0;
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "concatenate_clouds");

  ros::NodeHandle n;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("metricFile", metricFile);
  nhPrivate.getParam("trajFile", trajFile);
  nhPrivate.getParam("mapFile", mapFile);
  nhPrivate.getParam("overallMapVoxelSize", overallMapVoxelSize);
  nhPrivate.getParam("exploredAreaVoxelSize", exploredAreaVoxelSize);
  nhPrivate.getParam("exploredVolumeVoxelSize", exploredVolumeVoxelSize);
  nhPrivate.getParam("transInterval", transInterval);
  nhPrivate.getParam("yawInterval", yawInterval);
  nhPrivate.getParam("overallMapDisplayInterval", overallMapDisplayInterval);
  nhPrivate.getParam("exploredAreaDisplayInterval", exploredAreaDisplayInterval);

  ros::Rate loop_rate(10);

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/explored_areas", 5, laserCloudHandler);

  ros::spin();

  return (0);
}

*/