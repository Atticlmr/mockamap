#include "maps.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <random>
#include <vector>

#include <Eigen/Core>

#include "perlinnoise.hpp"

using namespace mocka;

void
Maps::randomMapGenerate()
{

  std::default_random_engine eng(info.seed);

  double _resolution = 1 / info.scale;

  double _x_l = -info.sizeX / (2 * info.scale);
  double _x_h = info.sizeX / (2 * info.scale);
  double _y_l = -info.sizeY / (2 * info.scale);
  double _y_h = info.sizeY / (2 * info.scale);
  double _h_l = 0;
  double _h_h = info.sizeZ / info.scale;

  double _w_l, _w_h;
  int    _ObsNum;

  info.node->get_parameter_or("width_min", _w_l, 0.6);
  info.node->get_parameter_or("width_max", _w_h, 1.5);
  info.node->get_parameter_or("obstacle_number", _ObsNum, 10);

  std::uniform_real_distribution<double> rand_x;
  std::uniform_real_distribution<double> rand_y;
  std::uniform_real_distribution<double> rand_w;
  std::uniform_real_distribution<double> rand_h;

  pcl::PointXYZ pt_random;

  rand_x = std::uniform_real_distribution<double>(_x_l, _x_h);
  rand_y = std::uniform_real_distribution<double>(_y_l, _y_h);
  rand_w = std::uniform_real_distribution<double>(_w_l, _w_h);
  rand_h = std::uniform_real_distribution<double>(_h_l, _h_h);

  for (int i = 0; i < _ObsNum; i++)
  {
    double x, y;
    x = rand_x(eng);
    y = rand_y(eng);

    double w, h;
    w = rand_w(eng);
    h = rand_h(eng);

    int widNum = ceil(w / _resolution);
    int heiNum = ceil(h / _resolution);

    int rl, rh, sl, sh;
    rl = -widNum / 2;
    rh = widNum / 2;
    sl = -widNum / 2;
    sh = widNum / 2;

    for (int r = rl; r < rh; r++)
      for (int s = sl; s < sh; s++)
      {
        for (int t = 0; t < heiNum; t++)
        {
          if ((r - rl) * (r - rh + 1) * (s - sl) * (s - sh + 1) * t *
                (t - heiNum + 1) ==
              0)
          {
            pt_random.x = x + r * _resolution;
            pt_random.y = y + s * _resolution;
            pt_random.z = t * _resolution;
            info.cloud->points.push_back(pt_random);
          }
        }
      }
  }

  info.cloud->width    = info.cloud->points.size();
  info.cloud->height   = 1;
  info.cloud->is_dense = true;

  pcl2ros();
}

void
Maps::randomCylinderRingMapGenerate()
{
  std::default_random_engine eng(info.seed);

  const double resolution = 1.0 / info.scale;
  const double x_l = -info.sizeX / (2.0 * info.scale);
  const double x_h = info.sizeX / (2.0 * info.scale);
  const double y_l = -info.sizeY / (2.0 * info.scale);
  const double y_h = info.sizeY / (2.0 * info.scale);

  int cylinder_num;
  int ring_num;
  double radius_min;
  double radius_max;
  double height_min;
  double height_max;
  double min_distance;
  double ring_radius_min;
  double ring_radius_max;
  double ring_z_min;
  double ring_z_max;
  double ring_max_yaw;

  info.node->get_parameter_or("cylinder_number", cylinder_num, 70);
  info.node->get_parameter_or("cylinder_radius_min", radius_min, 0.5);
  info.node->get_parameter_or("cylinder_radius_max", radius_max, 0.7);
  info.node->get_parameter_or("cylinder_height_min", height_min, 2.0);
  info.node->get_parameter_or("cylinder_height_max", height_max, 3.0);
  info.node->get_parameter_or("min_distance", min_distance, 1.4);
  info.node->get_parameter_or("ring_number", ring_num, 10);
  info.node->get_parameter_or("ring_radius_min", ring_radius_min, 0.7);
  info.node->get_parameter_or("ring_radius_max", ring_radius_max, 1.2);
  info.node->get_parameter_or("ring_z_min", ring_z_min, 0.7);
  info.node->get_parameter_or("ring_z_max", ring_z_max, 0.8);
  info.node->get_parameter_or("ring_max_yaw", ring_max_yaw, 0.5);

  const auto overrides =
    info.node->get_node_parameters_interface()->get_parameter_overrides();
  const auto has_override = [&overrides](const std::string& name) {
    return overrides.find(name) != overrides.end();
  };

  if (has_override("obstacle_number"))
  {
    info.node->get_parameter("obstacle_number", cylinder_num);
  }
  if (has_override("width_min"))
  {
    info.node->get_parameter("width_min", radius_min);
  }
  if (has_override("width_max"))
  {
    info.node->get_parameter("width_max", radius_max);
  }
  if (has_override("numGates"))
  {
    info.node->get_parameter("numGates", ring_num);
  }

  double gate_size;
  double max_angle_deg;
  info.node->get_parameter_or("gateSize", gate_size, 0.0);
  info.node->get_parameter_or("maxAngle", max_angle_deg, 0.0);
  if (has_override("gateSize") && gate_size > 0.0)
  {
    ring_radius_min = 0.5 * gate_size;
    ring_radius_max = 0.5 * gate_size;
  }
  constexpr double pi = 3.141592653589793;
  if (has_override("maxAngle") && max_angle_deg > 0.0)
  {
    ring_max_yaw = max_angle_deg * pi / 180.0;
  }

  std::uniform_real_distribution<double> rand_x(x_l, x_h);
  std::uniform_real_distribution<double> rand_y(y_l, y_h);
  std::uniform_real_distribution<double> rand_radius(radius_min, radius_max);
  std::uniform_real_distribution<double> rand_height(height_min, height_max);
  std::uniform_real_distribution<double> rand_inflation(0.5, 1.5);
  std::uniform_real_distribution<double> rand_ring_radius(ring_radius_min, ring_radius_max);
  std::uniform_real_distribution<double> rand_ring_radius2(ring_radius_min, ring_radius_max);
  std::uniform_real_distribution<double> rand_ring_z(ring_z_min, ring_z_max);
  std::uniform_real_distribution<double> rand_ring_yaw(-ring_max_yaw, ring_max_yaw);

  pcl::PointXYZ pt;
  std::vector<Eigen::Vector2d> cylinder_centers;

  int attempts = 0;
  const int max_attempts = std::max(1000, cylinder_num * 100);
  for (int i = 0; i < cylinder_num && attempts < max_attempts; ++i, ++attempts)
  {
    double x = rand_x(eng);
    double y = rand_y(eng);

    bool too_close = false;
    for (const auto& center : cylinder_centers)
    {
      if ((Eigen::Vector2d(x, y) - center).norm() < min_distance)
      {
        too_close = true;
        break;
      }
    }
    if (too_close)
    {
      --i;
      continue;
    }

    cylinder_centers.emplace_back(x, y);

    x = floor(x / resolution) * resolution + resolution / 2.0;
    y = floor(y / resolution) * resolution + resolution / 2.0;

    const double radius = 0.5 * rand_radius(eng) * rand_inflation(eng);
    const double height = rand_height(eng);
    const int wid_num = ceil((2.0 * radius) / resolution);
    const int hei_num = ceil(height / resolution);

    for (int r = -wid_num / 2; r < wid_num / 2; ++r)
    {
      for (int s = -wid_num / 2; s < wid_num / 2; ++s)
      {
        const double temp_x = x + (r + 0.5) * resolution;
        const double temp_y = y + (s + 0.5) * resolution;
      if ((Eigen::Vector2d(temp_x, temp_y) - Eigen::Vector2d(x, y)).norm() > radius)
        {
          continue;
        }

        for (int t = 0; t < hei_num; ++t)
        {
          pt.x = temp_x;
          pt.y = temp_y;
          pt.z = (t + 0.5) * resolution;
          info.cloud->points.push_back(pt);
        }
      }
    }
  }

  for (int i = 0; i < ring_num; ++i)
  {
    double x = floor(rand_x(eng) / resolution) * resolution + resolution / 2.0;
    double y = floor(rand_y(eng) / resolution) * resolution + resolution / 2.0;
    double z = floor(rand_ring_z(eng) / resolution) * resolution + resolution / 2.0;

    const double yaw = rand_ring_yaw(eng);
    Eigen::Matrix3d rotate;
    rotate << cos(yaw), -sin(yaw), 0.0,
              sin(yaw),  cos(yaw), 0.0,
              0.0,       0.0,      1.0;

    const double radius1 = rand_ring_radius(eng);
    const double radius2 = rand_ring_radius2(eng);

    constexpr double two_pi = 6.283185307179586;
    for (double angle = 0.0; angle < two_pi; angle += resolution / 2.0)
    {
      Eigen::Vector3d local_pt(0.0, radius1 * cos(angle), radius2 * sin(angle));
      Eigen::Vector3d world_pt = rotate * local_pt + Eigen::Vector3d(x, y, z);
      pt.x = world_pt(0);
      pt.y = world_pt(1);
      pt.z = world_pt(2);
      info.cloud->points.push_back(pt);
    }
  }

  info.cloud->width = info.cloud->points.size();
  info.cloud->height = 1;
  info.cloud->is_dense = true;

  RCLCPP_INFO(info.node->get_logger(),
              "generated random cylinder/ring map: cylinders=%d rings=%d points=%d",
              cylinder_num,
              ring_num,
              info.cloud->width);
  pcl2ros();
}

void
Maps::pcl2ros()
{
  // Convert PCL point cloud to ROS2 PointCloud2 message
  info.output->height = info.cloud->height;
  info.output->width = info.cloud->width;
  info.output->is_dense = info.cloud->is_dense;
  info.output->is_bigendian = false;
  info.output->header.frame_id = "map";
  
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
  
  info.output->fields.push_back(field_x);
  info.output->fields.push_back(field_y);
  info.output->fields.push_back(field_z);
  
  info.output->point_step = 12;  // 3 * float32
  info.output->row_step = info.output->point_step * info.output->width;
  
  // Copy data
  info.output->data.resize(info.cloud->points.size() * info.output->point_step);
  for (size_t i = 0; i < info.cloud->points.size(); ++i) {
    float x = info.cloud->points[i].x;
    float y = info.cloud->points[i].y;
    float z = info.cloud->points[i].z;
    memcpy(&info.output->data[i * info.output->point_step], &x, sizeof(float));
    memcpy(&info.output->data[i * info.output->point_step + 4], &y, sizeof(float));
    memcpy(&info.output->data[i * info.output->point_step + 8], &z, sizeof(float));
  }
  
  RCLCPP_INFO(info.node->get_logger(), "finish: infill %lf%%",
           info.cloud->width / (1.0 * info.sizeX * info.sizeY * info.sizeZ));
}

void
Maps::perlin3D()
{
  double complexity;
  double fill;
  int    fractal;
  double attenuation;

  info.node->get_parameter_or("complexity", complexity, 0.142857);
  info.node->get_parameter_or("fill", fill, 0.38);
  info.node->get_parameter_or("fractal", fractal, 1);
  info.node->get_parameter_or("attenuation", attenuation, 0.5);

  info.cloud->width  = info.sizeX * info.sizeY * info.sizeZ;
  info.cloud->height = 1;
  info.cloud->points.resize(info.cloud->width * info.cloud->height);

  PerlinNoise noise(info.seed);

  std::vector<double>* v = new std::vector<double>;
  v->reserve(info.cloud->width);
  for (int i = 0; i < info.sizeX; ++i)
  {
    for (int j = 0; j < info.sizeY; ++j)
    {
      for (int k = 0; k < info.sizeZ; ++k)
      {
        double tnoise = 0;
        for (int it = 1; it <= fractal; ++it)
        {
          int    dfv = pow(2, it);
          double ta  = attenuation / it;
          tnoise += ta * noise.noise(dfv * i * complexity,
                                     dfv * j * complexity,
                                     dfv * k * complexity);
        }
        v->push_back(tnoise);
      }
    }
  }
  std::sort(v->begin(), v->end());
  int    tpos = info.cloud->width * (1 - fill);
  double tmp  = v->at(tpos);
  RCLCPP_INFO(info.node->get_logger(), "threshold: %lf", tmp);

  int pos = 0;
  for (int i = 0; i < info.sizeX; ++i)
  {
    for (int j = 0; j < info.sizeY; ++j)
    {
      for (int k = 0; k < info.sizeZ; ++k)
      {
        double tnoise = 0;
        for (int it = 1; it <= fractal; ++it)
        {
          int    dfv = pow(2, it);
          double ta  = attenuation / it;
          tnoise += ta * noise.noise(dfv * i * complexity,
                                     dfv * j * complexity,
                                     dfv * k * complexity);
        }
        if (tnoise > tmp)
        {
          info.cloud->points[pos].x =
            i / info.scale - info.sizeX / (2 * info.scale);
          info.cloud->points[pos].y =
            j / info.scale - info.sizeY / (2 * info.scale);
          info.cloud->points[pos].z = k / info.scale;
          pos++;
        }
      }
    }
  }
  info.cloud->width = pos;
  RCLCPP_INFO(info.node->get_logger(), "the number of points before optimization is %d", info.cloud->width);
  info.cloud->points.resize(info.cloud->width * info.cloud->height);
  pcl2ros();
}

void
Maps::recursiveDivision(int xl, int xh, int yl, int yh, Eigen::MatrixXi& maze)
{
  RCLCPP_INFO(info.node->get_logger(),
    "generating maze with width %d , height %d", xh - xl + 1, yh - yl + 1);

  if (xl < xh - 3 && yl < yh - 3)
  { // the remaining area is larger than or equal to 5*5, need to add both x
    // wall and y wall
    bool valid = false; // used to judge whether the wall selection is valid
    int  xm    = 0;
    int  ym    = 0;
    RCLCPP_INFO(info.node->get_logger(), "entered 5*5 mode");
    while (valid == false)
    {
      xm = (std::rand() % (xh - xl - 1) + xl +
            1); // generating random number between xl+1 and xh-1(pointless to
                // add a wall at the sides)
      ym = (std::rand() % (yh - yl - 1) + yl +
            1); // generating random number between yl+1 and yh-1(pointless to
                // add a wall at the sides) xm and ym are the coordinate of the
                // center of the cross wall
      if (xl - 1 >= 0)
      { // there is a point at xl-1,ym
        if (maze(xl - 1, ym) == 0)
        { // this is an opening,need to change random number
          continue;
        }
      }

      else if (xh + 1 <= maze.cols() - 1)
      { // there is a point at xh+1,ym
        if (maze(xh + 1, ym) == 0)
        { // this is an opening,need to change random number
          continue;
        }
      }

      else if (yl - 1 >= 0)
      { // there is a point at xm,yl-1
        if (maze(xm, yl - 1) == 0)
        { // this is an opening,need to change random number
          continue;
        }
      }

      else if (yh + 1 <= maze.rows() - 1)
      { // there is a point at xm,yh+1
        if (maze(xm, yh + 1) == 0)
        { // this is an opening,need to change random number
          continue;
        }
      }

      valid = true;

    } // xm and ym are now the valid coordinate of the center of the wall
    for (int i = xl; i <= xh; i++)
    {
      maze(i, ym) = 1;
    }
    for (int j = yl; j <= yh; j++)
    {
      maze(xm, j) = 1;
    } // adding walls around the center point
    int d1 = std::rand() % (xm - xl) + xl;
    int d2 = std::rand() % (xh - xm) + xm + 1;
    int d3 = std::rand() % (ym - yl) + yl;
    int d4 =
      std::rand() % (yh - ym) + ym + 1; // generating four possible door points

    int decision = std::rand() % 4; // random selection of three doors
    switch (decision)
    {
      case 0:
        maze(d1, ym) = 0;
        maze(d2, ym) = 0;
        maze(xm, d3) = 0;
        break;

      case 1:
        maze(d1, ym) = 0;
        maze(d2, ym) = 0;
        maze(xm, d4) = 0;
        break;

      case 2:
        maze(d2, ym) = 0;
        maze(xm, d3) = 0;
        maze(xm, d4) = 0;
        break;

      case 3:
        maze(d1, ym) = 0;
        maze(xm, d3) = 0;
        maze(xm, d4) = 0;
        break;
    } // the doors are opened for this cell
    if (yl - 1 >= 0)
    {
      if (maze(xm, yl - 1) == 0)
      {
        maze(xm, yl) = 0;
      }
    }

    if (yh + 1 <= maze.rows() - 1)
    {
      if (maze(xm, yh + 1) == 0)
      {
        maze(xm, yh) = 0;
      }
    }

    if (xl - 1 >= 0)
    {
      if (maze(xl - 1, ym) == 0)
      {
        maze(xl, ym) = 0;
      }
    }

    if (xh + 1 <= maze.cols() - 1)
    {
      if (maze(xh + 1, ym) == 0)
      {
        maze(xh, ym) = 0;
      }
    }

    std::cout << maze << std::endl;
    recursiveDivision(xl, xm - 1, yl, ym - 1, maze);
    recursiveDivision(xm + 1, xh, yl, ym - 1, maze);
    recursiveDivision(xl, xm - 1, ym + 1, yh, maze);
    recursiveDivision(xm + 1, xh, ym + 1, yh, maze);

    RCLCPP_INFO(info.node->get_logger(), "finished generating maze with width %d , height %d",
             xh - xl + 1,
             yh - yl + 1);
    std::cout << maze << std::endl;
    return;
  } // when the remaining area is larger than or equal to 5*5

  else if (xl < xh - 2 && yl < yh - 2)
  {
    bool valid     = false; // used to judge whether the wall selection is valid
    int  xm        = 0;
    int  ym        = 0;
    int  doorcount = 0;
    xm             = (std::rand() % (xh - xl - 1) + xl +
          1); // generating random number between xl+1 and xh-1(pointless to
                          // add a wall at the sides)
    ym =
      (std::rand() % (yh - yl - 1) + yl +
       1); // generating random number between yl+1 and yh-1(pointless to
            // add a wall at the sides)
            // xm and ym are now the valid coordinate of the center of the wall
    for (int i = xl; i <= xh; i++)
    {
      maze(i, ym) = 1;
    }
    for (int j = yl; j <= yh; j++)
    {
      maze(xm, j) = 1;
    } // adding walls around the center point
    if (yl - 1 >= 0)
    {
      if (maze(xm, yl - 1) == 0)
      {
        maze(xm, yl) = 0;
        doorcount++;
      }
    }

    if (yh + 1 <= maze.rows() - 1)
    {
      if (maze(xm, yh + 1) == 0)
      {
        maze(xm, yh) = 0;
        doorcount++;
      }
    }

    if (xl - 1 >= 0)
    {
      if (maze(xl - 1, ym) == 0)
      {
        maze(xl, ym) = 0;
        doorcount++;
      }
    }

    if (xh + 1 <= maze.cols() - 1)
    {
      if (maze(xh + 1, ym) == 0)
      {
        maze(xh, ym) = 0;
        doorcount++;
      }
    }

    int d1 = std::rand() % (xm - xl) + xl;
    int d2 = std::rand() % (xh - xm) + xm + 1;
    int d3 = std::rand() % (ym - yl) + yl;
    int d4 =
      std::rand() % (yh - ym) + ym + 1; // generating four possible door points

    int decision = std::rand() % 4; // random selection of three doors
    switch (decision)
    {
      case 0:
        maze(d1, ym) = 0;
        maze(d2, ym) = 0;
        maze(xm, d3) = 0;
        break;

      case 1:
        maze(d1, ym) = 0;
        maze(d2, ym) = 0;
        maze(xm, d4) = 0;
        break;

      case 2:
        maze(d2, ym) = 0;
        maze(xm, d3) = 0;
        maze(xm, d4) = 0;
        break;

      case 3:
        maze(d1, ym) = 0;
        maze(xm, d3) = 0;
        maze(xm, d4) = 0;
        break;
    } // the doors are opened for this cell
    std::cout << maze << std::endl;

    RCLCPP_INFO(info.node->get_logger(), "finished generating maze with width %d , height %d",
             xh - xl + 1,
             yh - yl + 1);
    std::cout << maze << std::endl;
    return;
  }

  else if (xl < xh - 1 && yl < yh - 2)
  { // the case of 3*4+
    RCLCPP_INFO(info.node->get_logger(), "entered 3*4+ mode");
    int doorcount = 0;
    int ym        = 0;
    for (int i = yl; i <= yh; i++)
    {
      maze(xl + 1, i) = 1;
    } // filling a center wall
    if (yl - 1 >= 0)
    {
      if (maze(xl + 1, yl - 1) == 0)
      {
        maze(xl + 1, yl) = 0;
        doorcount++;
      }
    }
    if (yh + 1 <= maze.rows() - 1)
    {
      if (maze(xl + 1, yh + 1) == 0)
      {
        maze(xl + 1, yh) = 0;
        doorcount++;
      }
    } // opening doors if the wall blocks the old doors
    if (doorcount == 0)
    {
      ym               = std::rand() % (yh - yl + 1) + yl;
      maze(xl + 1, ym) = 0;
    }
  } // the case of 4+*3
  //
  else if (xl < xh - 2 && yl < yh - 1)
  { // the case of 4+*3
    RCLCPP_INFO(info.node->get_logger(), "entered 4+*3 mode");
    int doorcount = 0;
    int xm        = 0;
    for (int i = xl; i <= xh; i++)
    {
      maze(i, yl + 1) = 1;
    } // filling a center wall
    if (xl - 1 >= 0)
    {
      if (maze(xl - 1, yl + 1) == 0)
      {
        maze(xl, yl + 1) = 0;
        doorcount++;
      }
    }
    if (xh + 1 <= maze.cols() - 1)
    {
      if (maze(xh + 1, yl + 1) == 0)
      {
        maze(xh, yl + 1) = 0;
        doorcount++;
      }
    } // opening doors if the wall blocks the old doors
    if (doorcount == 0)
    {
      xm               = std::rand() % (xh - xl + 1) + xl;
      maze(xm, yl + 1) = 0;
    }
  } // the case of 4+*3

  else if (xl < xh - 1 && yl < yh - 1)
  { // the case of 3*3
    maze(xl + 1, yl + 1) = 1;
    return;
  }
  else
  {
    RCLCPP_INFO(info.node->get_logger(), "finished generating maze with width %d , height %d",
             xh - xl + 1,
             yh - yl + 1);
    return;
  }
}

void
Maps::recursizeDivisionMaze(Eigen::MatrixXi& maze)
{
  //! @todo all bugs here...
  int sx = maze.rows();
  int sy = maze.cols();

  int px, py;

  if (sx > 5)
    px = (std::rand() % (sx - 3) + 1);
  else
    return;

  if (sy > 5)
    py = (std::rand() % (sy - 3) + 1);
  else
    return;

  RCLCPP_INFO(info.node->get_logger(), "debug %d %d %d %d", sx, sy, px, py);

  int x1, x2, y1, y2;

  if (px != 1)
    x1 = (std::rand() % (px - 1) + 1);
  else
    x1 = 1;

  if ((sx - px - 3) > 0)
    x2 = (std::rand() % (sx - px - 3) + px + 1);
  else
    x2 = px + 1;

  if (py != 1)
    y1 = (std::rand() % (py - 1) + 1);
  else
    y1 = 1;

  if ((sy - py - 3) > 0)
    y2 = (std::rand() % (sy - py - 3) + py + 1);
  else
    y2 = py + 1;
  RCLCPP_INFO(info.node->get_logger(), "%d %d %d %d", x1, x2, y1, y2);

  if (px != 1 && px != (sx - 2))
  {
    for (int i = 1; i < (sy - 1); ++i)
    {
      if (i != y1 && i != y2)
        maze(px, i) = 1;
    }
  }
  if (py != 1 && py != (sy - 2))
  {
    for (int i = 1; i < (sx - 1); ++i)
    {
      if (i != x1 && i != x2)
        maze(i, py) = 1;
    }
  }
  switch (std::rand() % 4)
  {
    case 0:
      maze(x1, py) = 1;
      break;
    case 1:
      maze(x2, py) = 1;
      break;
    case 2:
      maze(px, y1) = 1;
      break;
    case 3:
      maze(px, y2) = 1;
      break;
  }

  if (px > 2 && py > 2)
  {
    Eigen::MatrixXi sub = maze.block(0, 0, px + 1, py + 1);
    recursizeDivisionMaze(sub);
    maze.block(0, 0, px, py) = sub;
  }
  if (px > 2 && (sy - py - 1) > 2)
  {
    Eigen::MatrixXi sub = maze.block(0, py, px + 1, sy - py);
    recursizeDivisionMaze(sub);
    maze.block(0, py, px + 1, sy - py) = sub;
  }
  if (py > 2 && (sx - px - 1) > 2)
  {
    Eigen::MatrixXi sub = maze.block(px, 0, sx - px, py + 1);
    recursizeDivisionMaze(sub);
    maze.block(px, 0, sx - px, py + 1) = sub;
  }
  if ((sx - px - 1) > 2 && (sy - py - 1) > 2)
  {

    Eigen::MatrixXi sub = maze.block(px, py, sy - px, sy - py);

    recursizeDivisionMaze(sub);
    maze.block(px, py, sy - px, sy - py) = sub;
  }
}

void
Maps::maze2D()
{
  double width;
  int    type;
  int    addWallX;
  int    addWallY;
  info.node->get_parameter_or("road_width", width, 1.0);
  info.node->get_parameter_or("add_wall_x", addWallX, 0);
  info.node->get_parameter_or("add_wall_y", addWallY, 0);
  info.node->get_parameter_or("maze_type", type, 1);

  int mx = info.sizeX / (width * info.scale);
  int my = info.sizeY / (width * info.scale);

  Eigen::MatrixXi maze(mx, my);
  maze.setZero();

  switch (type)
  {
    case 1:
      recursiveDivision(0, maze.cols() - 1, 0, maze.rows() - 1, maze);
      break;
  }

  if (addWallX)
  {
    for (int i = 0; i < mx; ++i)
    {
      maze(i, 0)      = 1;
      maze(i, my - 1) = 1;
    }
  }
  if (addWallY)
  {
    for (int i = 0; i < my; ++i)
    {
      maze(0, i)      = 1;
      maze(mx - 1, i) = 1;
    }
  }

  std::cout << maze << std::endl;

  for (int i = 0; i < mx; ++i)
  {
    for (int j = 0; j < my; ++j)
    {
      if (maze(i, j))
      {
        for (int ii = 0; ii < width * info.scale; ++ii)
        {
          for (int jj = 0; jj < width * info.scale; ++jj)
          {
            for (int k = 0; k < info.sizeZ; ++k)
            {
              pcl::PointXYZ pt_random;
              pt_random.x =
                i * width + ii / info.scale - info.sizeX / (2.0 * info.scale);
              pt_random.y =
                j * width + jj / info.scale - info.sizeY / (2.0 * info.scale);
              pt_random.z = k / info.scale;
              info.cloud->points.push_back(pt_random);
            }
          }
        }
      }
    }
  }
  info.cloud->width    = info.cloud->points.size();
  info.cloud->height   = 1;
  info.cloud->is_dense = true;
  pcl2ros();
}

Maps::BasicInfo
Maps::getInfo() const
{
  return info;
}

void
Maps::setInfo(const BasicInfo& value)
{
  info = value;
}

Maps::Maps()
{
}

void
Maps::generate(int type)
{
  switch (type)
  {
    default:
    case 1:
      perlin3D();
      break;
    case 2:
      randomMapGenerate();
      break;
    case 3:
      std::srand(info.seed);
      maze2D();
      break;
    case 4: // generating 3d maze
      std::srand(info.seed);
      Maze3DGen();
      break;
    case 5:
      randomCylinderRingMapGenerate();
      break;
  }
}

pcl::PointXYZ
MazePoint::getPoint()
{
  return point;
}

int
MazePoint::getPoint1()
{
  return point1;
}

int
MazePoint::getPoint2()
{
  return point2;
}

double
MazePoint::getDist1()
{
  return dist1;
}

double
MazePoint::getDist2()
{
  return dist2;
}

void
MazePoint::setPoint(pcl::PointXYZ p)
{
  point = p;
}

void
MazePoint::setPoint1(int p)
{
  point1 = p;
}

void
MazePoint::setPoint2(int p)
{
  point2 = p;
}

void
MazePoint::setDist1(double set)
{
  dist1 = set;
}

void
MazePoint::setDist2(double set)
{
  dist2 = set;
}

void
Maps::Maze3DGen()
{
  // getting required info parameters from the given node
  int    numNodes;
  double connectivity;
  int    nodeRad;
  int    roadRad;

  info.node->get_parameter_or("numNodes", numNodes, 10);
  info.node->get_parameter_or("connectivity", connectivity, 0.5);
  info.node->get_parameter_or("nodeRad", nodeRad, 3);
  info.node->get_parameter_or("roadRad", roadRad, 2);
  RCLCPP_INFO(info.node->get_logger(), "received parameters : numNodes: %d connectivity: "
           "%f nodeRad: %d roadRad: %d",
           numNodes,
           connectivity,
           nodeRad,
           roadRad);
  // generating random points
  std::vector<pcl::PointXYZ> base;

  for (int i = 0; i < numNodes; i++)
  {
    double rx = std::rand() / RAND_MAX +
                (std::rand() % info.sizeX) / info.scale -
                info.sizeX / (2 * info.scale);
    double ry = std::rand() / RAND_MAX +
                (std::rand() % info.sizeY) / info.scale -
                info.sizeY / (2 * info.scale);
    double rz = std::rand() / RAND_MAX +
                (std::rand() % info.sizeZ) / info.scale -
                info.sizeZ / (2 * info.scale);
    RCLCPP_INFO(info.node->get_logger(), "point: x: %f , y: %f , z: %f", rx, ry, rz);

    pcl::PointXYZ pt_random;
    pt_random.x = rx;
    pt_random.y = ry;
    pt_random.z = rz;
    base.push_back(pt_random);
  } // generating random cores in the space

  for (int i = 0; i < info.sizeX; i++)
  {
    for (int j = 0; j < info.sizeY; j++)
    {
      for (int k = 0; k < info.sizeZ; k++)
      { // for every scaled coordinate points
        pcl::PointXYZ test;
        test.x = i / info.scale - info.sizeX / (2 * info.scale);
        test.y = j / info.scale - info.sizeY / (2 * info.scale);
        test.z = k / info.scale -
                 info.sizeZ /
                   (2 * info.scale); // marking the corresponding point location

        MazePoint mp;
        mp.setPoint(test);
        mp.setPoint2(-1);
        mp.setPoint1(-1);
        mp.setDist1(10000.0);
        mp.setDist2(100000.0); // setting super large starting values
        for (int ii = 0; ii < numNodes; ii++)
        {
          double dist =
            std::sqrt((base[ii].x - test.x) * (base[ii].x - test.x) +
                      (base[ii].y - test.y) * (base[ii].y - test.y) +
                      (base[ii].z - test.z) * (base[ii].z - test.z));
          if (dist < mp.getDist1())
          {

            mp.setDist2(mp.getDist1());
            mp.setDist1(dist);

            mp.setPoint2(mp.getPoint1());
            mp.setPoint1(ii);
          }
          else if (dist < mp.getDist2())
          {
            mp.setDist2(dist);
            mp.setPoint2(ii);
          } // finding the distances to the nearest two cores
        }
        if (std::abs(mp.getDist2() - mp.getDist1()) < 1 / info.scale)
        { // the tested location is on one of the middle planes
          if ((mp.getPoint1() + mp.getPoint2()) >
                int((1 - connectivity) * numNodes) &&
              (mp.getPoint1() + mp.getPoint2()) <
                int((1 + connectivity) * numNodes))
          { // this is a holed wall
            double judge =
              std::sqrt((base[mp.getPoint1()].x - base[mp.getPoint2()].x) *
                          (base[mp.getPoint1()].x - base[mp.getPoint2()].x) +
                        (base[mp.getPoint1()].y - base[mp.getPoint2()].y) *
                          (base[mp.getPoint1()].y - base[mp.getPoint2()].y) +
                        (base[mp.getPoint1()].z - base[mp.getPoint2()].z) *
                          (base[mp.getPoint1()].z - base[mp.getPoint2()].z));
            if (mp.getDist1() + mp.getDist2() - judge >=
                roadRad / (info.scale * 3))
            {
              info.cloud->points.push_back(mp.getPoint());
            }
          }
          else
          {
            info.cloud->points.push_back(mp.getPoint());
          }
        }
      }
    }
  }

  info.cloud->width  = info.cloud->points.size();
  info.cloud->height = 1;
  RCLCPP_INFO(info.node->get_logger(), "the number of points before optimization is %d", info.cloud->width);
  info.cloud->points.resize(info.cloud->width * info.cloud->height);
  pcl2ros();
}
