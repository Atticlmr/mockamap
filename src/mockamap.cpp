#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <algorithm>
#include <cstring>
#include <iostream>
#include <vector>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>

#include "maps.hpp"

void
optimizeMap(mocka::Maps::BasicInfo& in)
{
  std::vector<int>* temp = new std::vector<int>;

  pcl::KdTreeFLANN<pcl::PointXYZ>     kdtree;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  cloud->width  = in.cloud->width;
  cloud->height = in.cloud->height;
  cloud->points.resize(cloud->width * cloud->height);

  for (int i = 0; i < cloud->width; i++)
  {
    cloud->points[i].x = in.cloud->points[i].x;
    cloud->points[i].y = in.cloud->points[i].y;
    cloud->points[i].z = in.cloud->points[i].z;
  }

  kdtree.setInputCloud(cloud);
  double radius = 1.75 / in.scale; // 1.75 is the rounded up value of sqrt(3)

  for (int i = 0; i < cloud->width; i++)
  {
    std::vector<int>   pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    if (kdtree.radiusSearch(cloud->points[i], radius, pointIdxRadiusSearch,
                            pointRadiusSquaredDistance) >= 27)
    {
      temp->push_back(i);
    }
  }
  for (int i = temp->size() - 1; i >= 0; i--)
  {
    in.cloud->points.erase(in.cloud->points.begin() +
                           temp->at(i)); // erasing the enclosed points
  }
  in.cloud->width -= temp->size();

  // Manual conversion from PCL to ROS2 PointCloud2
  in.output->height = in.cloud->height;
  in.output->width = in.cloud->width;
  in.output->is_dense = in.cloud->is_dense;
  in.output->is_bigendian = false;
  in.output->header.frame_id = "map";
  
  // Set up point fields (x, y, z)
  sensor_msgs::msg::PointField field_x, field_y, field_z;
  field_x.name = "x";
  field_x.offset = 0;
  field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field_x.count = 1;
  
  field_y.name = "y";
  field_y.offset = 4;
  field_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field_y.count = 1;
  
  field_z.name = "z";
  field_z.offset = 8;
  field_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field_z.count = 1;
  
  in.output->fields.clear();
  in.output->fields.push_back(field_x);
  in.output->fields.push_back(field_y);
  in.output->fields.push_back(field_z);
  
  in.output->point_step = 12;  // 3 * float32
  in.output->row_step = in.output->point_step * in.output->width;
  
  // Copy data
  in.output->data.resize(in.cloud->points.size() * in.output->point_step);
  for (size_t i = 0; i < in.cloud->points.size(); ++i) {
    float x = in.cloud->points[i].x;
    float y = in.cloud->points[i].y;
    float z = in.cloud->points[i].z;
    memcpy(&in.output->data[i * in.output->point_step], &x, sizeof(float));
    memcpy(&in.output->data[i * in.output->point_step + 4], &y, sizeof(float));
    memcpy(&in.output->data[i * in.output->point_step + 8], &z, sizeof(float));
  }
  
  RCLCPP_INFO(in.node->get_logger(), "finish: number of points after optimization %d", in.cloud->width);
  delete temp;
  return;
}

int
main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("mockamap");

  auto pcl_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("mock_map", 1);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  sensor_msgs::msg::PointCloud2       output;
  // Fill in the cloud data

  int seed;

  int sizeX;
  int sizeY;
  int sizeZ;

  double scale;
  double update_freq;

  int type;

  // Declare parameters with default values
  node->declare_parameter<int>("seed", 4546);
  node->declare_parameter<double>("update_freq", 1.0);
  node->declare_parameter<double>("resolution", 0.38);
  node->declare_parameter<int>("x_length", 100);
  node->declare_parameter<int>("y_length", 100);
  node->declare_parameter<int>("z_length", 10);
  node->declare_parameter<int>("type", 1);
  
  // Perlin noise 3D parameters (type=1)
  node->declare_parameter<double>("complexity", 0.142857);
  node->declare_parameter<double>("fill", 0.38);
  node->declare_parameter<int>("fractal", 1);
  node->declare_parameter<double>("attenuation", 0.5);
  
  // Random obstacles parameters (type=2)
  node->declare_parameter<double>("width_min", 0.6);
  node->declare_parameter<double>("width_max", 1.5);
  node->declare_parameter<int>("obstacle_number", 10);
  
  // 2D maze parameters (type=3)
  node->declare_parameter<double>("road_width", 1.0);
  node->declare_parameter<int>("add_wall_x", 0);
  node->declare_parameter<int>("add_wall_y", 0);
  node->declare_parameter<int>("maze_type", 1);
  
  // 3D maze parameters (type=4)
  node->declare_parameter<int>("numNodes", 10);
  node->declare_parameter<double>("connectivity", 0.5);
  node->declare_parameter<int>("nodeRad", 3);
  node->declare_parameter<int>("roadRad", 2);

  node->get_parameter("seed", seed);
  node->get_parameter("update_freq", update_freq);
  node->get_parameter("resolution", scale);
  node->get_parameter("x_length", sizeX);
  node->get_parameter("y_length", sizeY);
  node->get_parameter("z_length", sizeZ);
  node->get_parameter("type", type);

  scale = 1 / scale;
  sizeX = sizeX * scale;
  sizeY = sizeY * scale;
  sizeZ = sizeZ * scale;

  mocka::Maps::BasicInfo info;
  info.node = node;
  info.sizeX      = sizeX;
  info.sizeY      = sizeY;
  info.sizeZ      = sizeZ;
  info.seed       = seed;
  info.scale      = scale;
  info.output     = &output;
  info.cloud      = &cloud;

  mocka::Maps map;
  map.setInfo(info);
  map.generate(type);

  //  optimizeMap(info);

  //! @note publish loop
  rclcpp::Rate loop_rate(update_freq);
  while (rclcpp::ok())
  {
    pcl_pub->publish(output);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
