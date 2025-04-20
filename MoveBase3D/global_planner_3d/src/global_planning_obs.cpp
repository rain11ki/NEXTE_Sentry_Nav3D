#include <tf/transform_listener.h>
#include "backward.hpp"
#include "execution_classes.h"
#include <pcl/filters/passthrough.h>
#include <std_msgs/Float32MultiArray.h>
#include <pcl/filters/voxel_grid.h> // 包含VoxelGrid滤波器的头文件
#include <pcl/common/common.h>
using namespace std;
using namespace Eigen;
using namespace EXECUTION;

namespace backward
{
  backward::SignalHandling sh;
}
World *world = NULL;
// ros related
ros::Subscriber pt_sub;
ros::Subscriber world_sub;
ros::Publisher obs_pub;
ros::Publisher obs_cost_pub;
ros::Publisher obs_array_pub;

// 参数
double resolution, leaf_size, local_x_l, local_x_u, local_y_l, local_y_u, local_z_l, local_z_u;
string map_frame_id;
string lidar_frame_id;
string base_frame_id;

double expansionCoefficient = 1;
double expansion = 1;

tf::TransformListener *listener_ptr;
sensor_msgs::PointCloud2 worldPoints;
pcl::PointCloud<pcl::PointXYZ> worldCloud;

void rcvLidarCallBack(const sensor_msgs::PointCloud2 &lidar_points);

void rcvLidarCallBack(const sensor_msgs::PointCloud2 &lidar_points)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(lidar_points, *cloud);
  //printf("callback");
  // 设置 VoxelGrid 滤波器
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setInputCloud(cloud);
  voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_VoxelGrid(new pcl::PointCloud<pcl::PointXYZ>);
  voxel_grid.filter(*cloud_after_VoxelGrid);

  // 过滤到指定范围内的点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_PassThrough(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> passthrough;
  passthrough.setInputCloud(cloud_after_VoxelGrid);
  passthrough.setFilterFieldName("x");
  passthrough.setFilterLimits(local_x_l, local_x_u);
  passthrough.filter(*cloud_after_PassThrough);

  passthrough.setInputCloud(cloud_after_PassThrough);
  passthrough.setFilterFieldName("y");
  passthrough.setFilterLimits(local_y_l, local_y_u);
  passthrough.filter(*cloud_after_PassThrough);

  passthrough.setInputCloud(cloud_after_PassThrough);
  passthrough.setFilterFieldName("z");
  passthrough.setFilterLimits(local_z_l, local_z_u);
  passthrough.filter(*cloud_after_PassThrough);

  // 新建一个新的局部World 即:3D Map
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filt(new pcl::PointCloud<pcl::PointXYZ>);

  Vector3d lowerbound(local_x_l, local_y_l, local_z_l);
  Vector3d upperbound(local_x_u, local_y_u, local_z_u);

  World local_world = World(resolution);
  local_world.initGridMap(lowerbound, upperbound);

  // 填充局部World 即:3D Map
  for (const auto &pt : (*cloud_after_PassThrough).points)
  {
    Vector3d obstacle(pt.x, pt.y, pt.z);
    if (local_world.isFree(obstacle))
    {
      local_world.setObs(obstacle);
      Vector3d obstacle_round = local_world.coordRounding(obstacle);
      pcl::PointXYZ pt_add;
      pt_add.x = obstacle_round(0);
      pt_add.y = obstacle_round(1);
      pt_add.z = obstacle_round(2);
      cloud_filt->points.push_back(pt_add);
    }
  }

  listener_ptr->waitForTransform(map_frame_id, base_frame_id, ros::Time(0), ros::Duration(2.0));
  
  for (const auto &pt : (*cloud_after_PassThrough).points)
  {
    Vector3d obstacle(pt.x, pt.y, pt.z);
    if (local_world.isFree(obstacle))
    {
      local_world.setObs(obstacle);
      Vector3d obstacle_round = local_world.coordRounding(obstacle);
      pcl::PointXYZ pt_add;
      pt_add.x = obstacle_round(0);
      pt_add.y = obstacle_round(1);
      pt_add.z = obstacle_round(2);
      cloud_filt->points.push_back(pt_add);
    }
  }

  listener_ptr->waitForTransform(map_frame_id, base_frame_id, ros::Time(0), ros::Duration(2.0));
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tran(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tran_cost(new pcl::PointCloud<pcl::PointXYZ>);

  std_msgs::Float32MultiArray obs_array;
  std::unordered_map<std::string, pcl::PointXYZ> voxel_map;
  float half_res = resolution / 2;
  for (const auto &pt : worldCloud) {
      // 使用体素中心作为键
      std::string key = std::to_string(static_cast<int>(pt.x / resolution)) + "_" + 
                       std::to_string(static_cast<int>(pt.y / resolution));
      voxel_map[key] = pt;
  }
  
  for (const auto &pt : cloud_filt->points) {
      geometry_msgs::PointStamped origin_point;
      origin_point.header.frame_id = lidar_frame_id;
      origin_point.header.stamp = ros::Time();
      origin_point.point.x = pt.x;
      origin_point.point.y = pt.y;
      origin_point.point.z = pt.z;
  
      geometry_msgs::PointStamped trans_point;
      listener_ptr->transformPoint(map_frame_id, origin_point, trans_point);
  
      pcl::PointXYZ _pt;
      _pt.x = trans_point.point.x;
      _pt.y = trans_point.point.y;
      _pt.z = trans_point.point.z;
  
      // 计算当前点所在的体素键
      std::string current_key = std::to_string(static_cast<int>(_pt.x / resolution)) + "_" + 
                              std::to_string(static_cast<int>(_pt.y / resolution));
  
      // 查找是否存在于体素地图中
      auto it = voxel_map.find(current_key);
      if (it != voxel_map.end()) {
          const auto& voxel_center = it->second;
          float z_diff = _pt.z - voxel_center.z;
          
          // 使用浮点数近似比较代替精确比较
          if (z_diff >= resolution * 3 - 1e-6) {
              // 添加主点
              cloud_tran_cost->points.push_back(_pt);
              cloud_tran->points.push_back(_pt);
              
              // 预计算所有扩展点
              std::array<pcl::PointXYZ, 8> expanded_pts = {
                  pcl::PointXYZ{_pt.x + expansion, _pt.y, _pt.z},
                  pcl::PointXYZ{_pt.x - expansion, _pt.y, _pt.z},
                  pcl::PointXYZ{_pt.x, _pt.y + expansion, _pt.z},
                  pcl::PointXYZ{_pt.x, _pt.y - expansion, _pt.z},
                  pcl::PointXYZ{_pt.x + expansion, _pt.y + expansion, _pt.z},
                  pcl::PointXYZ{_pt.x - expansion, _pt.y - expansion, _pt.z},
                  pcl::PointXYZ{_pt.x + expansion, _pt.y - expansion, _pt.z},
                  pcl::PointXYZ{_pt.x - expansion, _pt.y + expansion, _pt.z}
              };
              
              // 批量添加扩展点
              cloud_tran->points.insert(cloud_tran->points.end(), expanded_pts.begin(), expanded_pts.end());
              
              // 批量添加数据到obs_array
              obs_array.data.push_back(_pt.x);
              obs_array.data.push_back(_pt.y);
              obs_array.data.push_back(_pt.z);
              
              for (const auto& exp_pt : expanded_pts) {
                  obs_array.data.push_back(exp_pt.x);
                  obs_array.data.push_back(exp_pt.y);
                  obs_array.data.push_back(exp_pt.z);
              }
          }
      }
  }

  sensor_msgs::PointCloud2 obs_vis;
  pcl::toROSMsg(*cloud_tran, obs_vis);
  obs_vis.header.frame_id = map_frame_id;
  obs_pub.publish(obs_vis);

  obs_array_pub.publish(obs_array);

  sensor_msgs::PointCloud2 obs_cost;
  pcl::toROSMsg(*cloud_tran_cost, obs_cost);
  obs_cost.header.frame_id = map_frame_id;
  obs_cost_pub.publish(obs_cost);
}

void rcvWorldCallBack(const sensor_msgs::PointCloud2 &pointcloud_map);

/**
 *@brief receive point cloud to build the grid map
 */
void rcvWorldCallBack(const sensor_msgs::PointCloud2 &pointcloud_map)
{
  pcl::fromROSMsg(pointcloud_map, worldCloud);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "global_planning_obs");
  ros::NodeHandle nh("~");

  //pt_sub = nh.subscribe("/cloud_registered_body", 1, rcvLidarCallBack);
  world_sub = nh.subscribe("/map", 1, rcvWorldCallBack);

  obs_pub = nh.advertise<sensor_msgs::PointCloud2>("obs_vis", 1);
  obs_cost_pub = nh.advertise<sensor_msgs::PointCloud2>("obs_cost", 1);
  obs_array_pub = nh.advertise<std_msgs::Float32MultiArray>("/obs_raw", 1);

  nh.getParam("map/map_frame_id", map_frame_id);
  nh.getParam("map/lidar_frame_id", lidar_frame_id);
  nh.getParam("map/base_frame_id", base_frame_id);
  nh.param("map/resolution", resolution, 0.1);

  nh.param("map/expansionCoefficient", expansionCoefficient, 1.0);
  // 降采样过滤
  nh.param("map/leaf_size", leaf_size, 0.2);
  // 边界过滤
  nh.param("map/local_x_l", local_x_l, -2.0);
  nh.param("map/local_x_u", local_x_u, 2.0);
  nh.param("map/local_y_l", local_y_l, -2.0);
  nh.param("map/local_y_u", local_y_u, 2.0);
  nh.param("map/local_z_l", local_z_l, -0.3);
  nh.param("map/local_z_u", local_z_u, 0.5);

  expansion = resolution * expansionCoefficient;

  tf::TransformListener listener;
  listener_ptr = &listener;
  world = new World(resolution);

  while (ros::ok())
  {
    timeval start;
    gettimeofday(&start, NULL);
    ros::spinOnce();
    double ms;
    do
    {
      timeval end;
      gettimeofday(&end, NULL);
      ms = 1000 * (end.tv_sec - start.tv_sec) + 0.001 * (end.tv_usec - start.tv_usec);
    } while (ms < 50);
  }
  return 0;
}

// #include <tf/transform_listener.h>
// #include "backward.hpp"
// #include "execution_classes.h"
// #include <pcl/filters/passthrough.h>
// #include <std_msgs/Float32MultiArray.h>
// #include <pcl/filters/voxel_grid.h>
// #include <mutex>
// #include <unordered_map>

// using namespace std;
// using namespace Eigen;
// using namespace EXECUTION;

// namespace backward {
//   backward::SignalHandling sh;
// }

// World *world = NULL;
// // ros related
// ros::Subscriber pt_sub;
// ros::Subscriber world_sub;
// ros::Publisher obs_pub;
// ros::Publisher obs_cost_pub;
// ros::Publisher obs_array_pub;

// // 参数
// double resolution, leaf_size, local_x_l, local_x_u, local_y_l, local_y_u, local_z_l, local_z_u;
// string map_frame_id;
// string lidar_frame_id;
// string base_frame_id;

// double expansionCoefficient = 1;
// double expansion = 1;

// tf::TransformListener *listener_ptr;
// sensor_msgs::PointCloud2 worldPoints;
// pcl::PointCloud<pcl::PointXYZ> worldCloud;
// std::mutex world_cloud_mutex;  // 保护worldCloud的互斥锁

// void rcvLidarCallBack(const sensor_msgs::PointCloud2 &lidar_points);

// void rcvLidarCallBack(const sensor_msgs::PointCloud2 &lidar_points) {
//   //ROS_DEBUG("[Start] New point cloud processing");

//   // 1. Convert ROS message to PCL


//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::fromROSMsg(lidar_points, *cloud);
//   //ROS_INFO("[Input] Raw cloud size: %zu points", cloud->points.size());

//   // 2. Voxel Grid Filter
//   pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
//   voxel_grid.setInputCloud(cloud);
//   voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_VoxelGrid(new pcl::PointCloud<pcl::PointXYZ>);
//   voxel_grid.filter(*cloud_after_VoxelGrid);


//   // 3. PassThrough Filters
//   pcl::PassThrough<pcl::PointXYZ> passthrough;
  
//   passthrough.setInputCloud(cloud_after_VoxelGrid);
//   passthrough.setFilterFieldName("x");
//   passthrough.setFilterLimits(local_x_l, local_x_u);
//   passthrough.filter(*cloud_after_VoxelGrid);


//   passthrough.setFilterFieldName("y");
//   passthrough.setFilterLimits(local_y_l, local_y_u);
//   passthrough.filter(*cloud_after_VoxelGrid);


//   passthrough.setFilterFieldName("z");
//   passthrough.setFilterLimits(local_z_l, local_z_u);
//   passthrough.filter(*cloud_after_VoxelGrid);


//   // 4. Local World Construction
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filt(new pcl::PointCloud<pcl::PointXYZ>);
//   Vector3d lowerbound(local_x_l, local_y_l, local_z_l);
//   Vector3d upperbound(local_x_u, local_y_u, local_z_u);

//   World local_world(resolution);
//   local_world.initGridMap(lowerbound, upperbound);

//   int obs_count = 0;
//   for (const auto &pt : cloud_after_VoxelGrid->points) {
//     Vector3d obstacle(pt.x, pt.y, pt.z);
//     if (local_world.isFree(obstacle)) {
//       local_world.setObs(obstacle);
//       Vector3d obstacle_round = local_world.coordRounding(obstacle);
//       cloud_filt->points.emplace_back(obstacle_round[0], obstacle_round[1], obstacle_round[2]);
//       obs_count++;
//     }
//   }
//   //ROS_INFO("[World] Valid obstacles: %d (res: %.3fm)", obs_count, resolution);

//   // 5. TF Transform Setup

//   try {
//     listener_ptr->waitForTransform(map_frame_id, lidar_frame_id, ros::Time(0), ros::Duration(2.0));
//   } catch (tf::TransformException &ex) {
//     ROS_ERROR("[TF] Wait failed: %s", ex.what());
//     return;
//   }

//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tran(new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tran_cost(new pcl::PointCloud<pcl::PointXYZ>);
//   std_msgs::Float32MultiArray obs_array;

//   // 6. Build Voxel Hash Map
//   std::unordered_map<std::string, std::vector<Vector3d>> voxel_map;
//   {
//     std::lock_guard<std::mutex> lock(world_cloud_mutex);
//     ROS_DEBUG("[Voxel] World cloud size: %zu", worldCloud.points.size());
//     for (const auto &world_pt : worldCloud) {
//       double vx = floor(world_pt.x / resolution) * resolution + resolution / 2;
//       double vy = floor(world_pt.y / resolution) * resolution + resolution / 2;
//       string key = to_string(vx) + "_" + to_string(vy);
//       voxel_map[key].emplace_back(vx, vy, world_pt.z);
//     }
//   }
//   ROS_INFO("[Voxel] Hash map size: %zu", voxel_map.size());

//   // 7. Process Each Point
//   int transformed_count = 0;
//   int voxel_hits = 0;
//   int z_condition_passed = 0;
  
//   for (const auto &pt : cloud_filt->points) {
//     // Coordinate Transform
//     geometry_msgs::PointStamped origin_point;
//     origin_point.header.frame_id = lidar_frame_id;
//     origin_point.header.stamp = ros::Time();
//     origin_point.point.x = pt.x;
//     origin_point.point.y = pt.y;
//     origin_point.point.z = pt.z;

//     geometry_msgs::PointStamped trans_point;
//     try {
//       listener_ptr->transformPoint(map_frame_id, origin_point, trans_point);
//       transformed_count++;
//     } catch (tf::TransformException &ex) {
//       ROS_WARN("[TF] Point transform failed: %s (original: %.2f,%.2f,%.2f)", 
//               ex.what(), pt.x, pt.y, pt.z);
//       continue;
//     }

//     pcl::PointXYZ _pt;
//     _pt.x = trans_point.point.x;
//     _pt.y = trans_point.point.y;
//     _pt.z = trans_point.point.z;

//     // Voxel Query
//     double vx = floor(_pt.x / resolution) * resolution + resolution / 2;
//     double vy = floor(_pt.y / resolution) * resolution + resolution / 2;
//     string key = to_string(vx) + "_" + to_string(vy);

//     auto it = voxel_map.find(key);
//     if (it != voxel_map.end()) {
//       voxel_hits++;
//       for (const auto &voxel_pt : it->second) {
//         double z_diff = _pt.z - voxel_pt.z();
//         ROS_DEBUG("[Height] Check diff: %.3fm (current:%.3f, voxel:%.3f)", 
//                  z_diff, _pt.z, voxel_pt.z());
        
//         if (z_diff >= resolution * 3) {
//           z_condition_passed++;
//           ROS_DEBUG("[Match] Condition passed (key=%s)", key.c_str());

//           // Add expansion points
//           const auto add_expansion = [&](double dx, double dy) {
//             pcl::PointXYZ exp_pt;
//             exp_pt.x = _pt.x + dx * expansion;
//             exp_pt.y = _pt.y + dy * expansion;
//             exp_pt.z = _pt.z;
//             cloud_tran->push_back(exp_pt);
//             obs_array.data.insert(obs_array.data.end(), {exp_pt.x, exp_pt.y, exp_pt.z});
//           };

//           cloud_tran_cost->push_back(_pt);
//           cloud_tran->push_back(_pt);
//           obs_array.data.insert(obs_array.data.end(), {_pt.x, _pt.y, _pt.z});

//           // 8-direction expansion
//           add_expansion(1, 0);   // x+
//           add_expansion(-1, 0);  // x-
//           add_expansion(0, 1);   // y+
//           add_expansion(0, -1);  // y-
//           add_expansion(1, 1);   // Q1
//           add_expansion(-1, -1); // Q3
//           add_expansion(1, -1);  // Q4
//           add_expansion(-1, 1);  // Q2
//         }
//       }
//     }
//   }

//   ROS_INFO("[Stats] Transformed: %d, Voxel hits: %d, Z-passed: %d", 
//           transformed_count, voxel_hits, z_condition_passed);

//   // 8. Publish Results
//   if (!cloud_tran->empty()) {
//     sensor_msgs::PointCloud2 obs_vis;
//     pcl::toROSMsg(*cloud_tran, obs_vis);
//     obs_vis.header.frame_id = map_frame_id;
//     obs_pub.publish(obs_vis);
//     ROS_DEBUG("[Publish] Obstacle cloud: %zu points", cloud_tran->size());
//   } else {
//     ROS_WARN("[Publish] Empty obstacle cloud");
//   }

//   if (!obs_array.data.empty()) {
//     obs_array_pub.publish(obs_array);
//     ROS_DEBUG("[Publish] Obstacle array: %zu elements", obs_array.data.size()/3);
//   }

//   if (!cloud_tran_cost->empty()) {
//     sensor_msgs::PointCloud2 obs_cost;
//     pcl::toROSMsg(*cloud_tran_cost, obs_cost);
//     obs_cost.header.frame_id = map_frame_id;
//     obs_cost_pub.publish(obs_cost);
//   }
// }
// void rcvWorldCallBack(const sensor_msgs::PointCloud2 &pointcloud_map);

// /**
//  *@brief receive point cloud to build the grid map
//  */
// void rcvWorldCallBack(const sensor_msgs::PointCloud2 &pointcloud_map)
// {
//   pcl::fromROSMsg(pointcloud_map, worldCloud);
// }
// int main(int argc, char **argv) {
//   ros::init(argc, argv, "global_planning_obs");
//   ros::NodeHandle nh("~");

//   // 参数初始化
//   nh.getParam("map/map_frame_id", map_frame_id);
//   nh.getParam("map/lidar_frame_id", lidar_frame_id);
//   nh.getParam("map/base_frame_id", base_frame_id);
//   nh.param("map/resolution", resolution, 0.1);
//   nh.param("map/expansionCoefficient", expansionCoefficient, 1.0);
//   expansion = resolution * expansionCoefficient;

//   // 订阅和发布
//   pt_sub = nh.subscribe("/cloud_registered", 1, rcvLidarCallBack);
//   world_sub = nh.subscribe("map", 1, rcvWorldCallBack);
//   obs_pub = nh.advertise<sensor_msgs::PointCloud2>("obs_vis", 1);
//   obs_cost_pub = nh.advertise<sensor_msgs::PointCloud2>("obs_cost", 1);
//   obs_array_pub = nh.advertise<std_msgs::Float32MultiArray>("/obs_raw", 1);

//   // TF初始化
//   tf::TransformListener listener;
//   listener_ptr = &listener;
//   world = new World(resolution);

//   // 关键修复5: 使用异步Spinner
//   ros::AsyncSpinner spinner(2);
//   spinner.start();
//   ros::waitForShutdown();

//   delete world;
//   return 0;
// }