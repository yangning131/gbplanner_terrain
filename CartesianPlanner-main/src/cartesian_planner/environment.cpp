/***********************************************************************************
 *  C++ Source Codes for "Autonomous Driving on Curvy Roads without Reliance on
 *  Frenet Frame: A Cartesian-based Trajectory Planning Method".
 ***********************************************************************************
 *  Copyright (C) 2022 Bai Li
 *  Users are suggested to cite the following article when they use the source codes.
 *  Bai Li et al., "Autonomous Driving on Curvy Roads without Reliance on
 *  Frenet Frame: A Cartesian-based Trajectory Planning Method",
 *  IEEE Transactions on Intelligent Transportation Systems, 2022.
 ***********************************************************************************/

#include "cartesian_planner/environment.h"
#include "cartesian_planner/visualization/plot.h"

using namespace std;
using namespace Eigen;

namespace cartesian_planner {

constexpr double kSampleStep = 0.1;

void Environment::SetReference(const DiscretizedTrajectory &reference) {
  reference_ = reference;

  road_barrier_.clear();

  double start_s = reference_.data().front().s;
  double back_s = reference_.data().back().s;
  int sample_points = int((back_s - start_s) / kSampleStep);
  for (int i = 0; i <= sample_points; i++) {
    double s = start_s + i * kSampleStep;
    auto ref = reference_.EvaluateStation(s);

    road_barrier_.push_back(reference_.GetCartesian(s, ref.left_bound));
    road_barrier_.push_back(reference_.GetCartesian(s, -ref.right_bound));
  }

  std::sort(road_barrier_.begin(), road_barrier_.end(), [](const Vec2d &a, const Vec2d &b) {
    return a.x() < b.x();
  });
}

void Environment::SetReference(const nav_msgs::Path &reference_path) {
  reference_path_ = reference_path;
  
}
void Environment::SetObstacles_map(const nav_msgs::OccupancyGrid &obstacles_map)
{ 
  obstacles_map_ = obstacles_map;
}

//对象是const 里面的函数也应该是const修饰
bool Environment::CheckStaticCollision(const math::Box2d &rect) {
  // double xmax, ymax, xmin, ymin;
  double xmax = rect.center_x() + rect.half_length();
  double ymax = rect.center_y() + rect.half_width();
  double xmin = rect.center_x() - rect.half_length();
  double ymin = rect.center_y() - rect.half_width();
  // std::tie(xmax, xmin, ymax, ymin) = rect.Getboxposition();

  // for(double x = xmin ;x<=xmax ; x+=getObstacle_map().info.resolution) 
  // {     
  //       int index_x = (int)round((x - getObstacle_map().info.origin.position.x) / getObstacle_map().info.resolution);
  //       int index_y_max = (int)round((ymax - getObstacle_map().info.origin.position.y) / getObstacle_map().info.resolution);
  //       int index_y_min = (int)round((ymin - getObstacle_map().info.origin.position.y) / getObstacle_map().info.resolution);
  //       int index_0 = index_x + index_y_min * getObstacle_map().info.width;
  //       int index_1 = index_x + index_y_max * getObstacle_map().info.width;
  //       if (index_x < 0 || index_x >= getObstacle_map().info.width || index_y_min < 0 || index_y_min >= getObstacle_map().info.height||
  //           index_y_max < 0 || index_y_max >= getObstacle_map().info.height)    
  //           return true;
  //       if (getObstacle_map().data[index_0] != 0||getObstacle_map().data[index_1] != 0)
  //           return true;
  // }

  // for(double y = ymin ;y<=ymax ; y+=getObstacle_map().info.resolution)
  // {
  //       int index_x_min = (int)round((xmin - getObstacle_map().info.origin.position.x) / getObstacle_map().info.resolution);
  //       int index_x_max = (int)round((xmax - getObstacle_map().info.origin.position.x) / getObstacle_map().info.resolution);
  //       int index_y = (int)round((y - getObstacle_map().info.origin.position.y) / getObstacle_map().info.resolution);
  //       int index_0 = index_x_min + index_y * getObstacle_map().info.width;
  //       int index_1 = index_x_max + index_y * getObstacle_map().info.width;
  //       if (index_x_min < 0 || index_x_min >= getObstacle_map().info.width || index_x_max < 0 || index_x_max >= getObstacle_map().info.width||
  //           index_y < 0 || index_y >= getObstacle_map().info.height)
  //           return true;
  //       if (getObstacle_map().data[index_0] != 0||getObstacle_map().data[index_1] != 0)
  //           return true;
  // }
  //8989998
  // if(xmax-xmin>10.0)
  // std::cout<<"QWQWQWQWQWQ  "<<xmax-xmin<<"DDSDSDSDSDS  "<<ymax-ymin<<std::endl;
  for(double x = xmin ;x<=xmax ; x+=getObstacle_map().info.resolution) 
  {     
        int index_x = (int)round((x - getObstacle_map().info.origin.position.x) / getObstacle_map().info.resolution);
        int index_y_min = (int)round((ymin - getObstacle_map().info.origin.position.y) / getObstacle_map().info.resolution);
        int index_0 = index_x + index_y_min * getObstacle_map().info.width;
        if (index_x < 0 || index_x >= getObstacle_map().info.width || index_y_min < 0 || index_y_min >= getObstacle_map().info.height)    
             continue;
        if (getObstacle_map().data[index_0] != 0)
            return true;
  }

  for(double y = ymin ;y<=ymax ; y+=getObstacle_map().info.resolution)
  {
        int index_x_min = (int)round((xmin - getObstacle_map().info.origin.position.x) / getObstacle_map().info.resolution);
        int index_y = (int)round((y - getObstacle_map().info.origin.position.y) / getObstacle_map().info.resolution);
        int index_0 = index_x_min + index_y * getObstacle_map().info.width;
        if (index_x_min < 0 || index_x_min >= getObstacle_map().info.width || 
            index_y < 0 || index_y >= getObstacle_map().info.height)
            continue;
        if (getObstacle_map().data[index_0] != 0)
            return true;
  }
  for(double x = xmin ;x<=xmax ; x+=getObstacle_map().info.resolution) 
  {     
        int index_x = (int)round((x - getObstacle_map().info.origin.position.x) / getObstacle_map().info.resolution);
        int index_y_max = (int)round((ymax - getObstacle_map().info.origin.position.y) / getObstacle_map().info.resolution);
        int index_1 = index_x + index_y_max * getObstacle_map().info.width;
        if (index_x < 0 || index_x >= getObstacle_map().info.width ||
            index_y_max < 0 || index_y_max >= getObstacle_map().info.height)    
            continue;
        if (getObstacle_map().data[index_1] != 0)
            return true;
  }

  for(double y = ymin ;y<=ymax ; y+=getObstacle_map().info.resolution)
  {
        int index_x_max = (int)round((xmax - getObstacle_map().info.origin.position.x) / getObstacle_map().info.resolution);
        int index_y = (int)round((y - getObstacle_map().info.origin.position.y) / getObstacle_map().info.resolution);
        int index_1 = index_x_max + index_y * getObstacle_map().info.width;
        if ( index_x_max < 0 || index_x_max >= getObstacle_map().info.width||
            index_y < 0 || index_y >= getObstacle_map().info.height)
            continue;
        if (getObstacle_map().data[index_1] != 0)
            return true;
  }
// std::cout<<"getObstacle_map().info.resolution"<<getObstacle_map().info.resolution<<std::endl;
// for(double x = xmin ;x<=xmax ; x+=getObstacle_map().info.resolution)
// {
//     for(double y = ymin ;y<=ymax ; y+=getObstacle_map().info.resolution)
//   {
//         int index_x_max = (int)round((x - getObstacle_map().info.origin.position.x) / getObstacle_map().info.resolution);
//         int index_y = (int)round((y - getObstacle_map().info.origin.position.y) / getObstacle_map().info.resolution);
//         int index_1 = index_x_max + index_y * getObstacle_map().info.width;



//         if ( index_x_max < 0 || index_x_max >= getObstacle_map().info.width||
//             index_y < 0 || index_y >= getObstacle_map().info.height)
//             continue;
//         if (getObstacle_map().data[index_1]>0)
//             return true;
//   }
// }
            // math::AABox2d box({rect.center_x()-0.2, rect.center_y()-0.2}, {rect.center_x()+0.2, rect.center_y()+0.2});

            //     visualization::PlotPolygon(math::Polygon2d(math::Box2d(box)), 0.02, visualization::Color::Grey, 0,
            //                    "Front Corridor1");

  

  //  for (auto &obstacle: obstacles_) {
  //   if (obstacle.HasOverlap(rect)) {
  //     return true;
  //   }
  // }


  return false;
}
  // for (auto &obstacle: obstacles_) {
  //   if (obstacle.HasOverlap(rect)) {
  //     return true;
  //   }
  // }

  // if (road_barrier_.empty()) {
  //   return false;
  // }

  // if (rect.max_x() < road_barrier_.front().x() || rect.min_x() > road_barrier_.back().x()) {
  //   return false;
  // }

  // auto comp = [](double val, const Vec2d &a) {
  //   return val < a.x();
  // };

  // binary search

  // auto check_start = std::upper_bound(road_barrier_.begin(), road_barrier_.end(), rect.min_x(), comp);
  // auto check_end = std::upper_bound(road_barrier_.begin(), road_barrier_.end(), rect.max_x(), comp);

  // if (check_start > road_barrier_.begin()) {
  //   std::advance(check_start, -1);
  // }

  // for (auto iter = check_start; iter != check_end; iter++) {
  //   if (rect.IsPointIn(*iter)) {
  //     return true;
  //   }
  // }

bool Environment::CheckCollision(double time, const math::Box2d &rect) {
  if (CheckDynamicCollision(time, rect)) {
    return true;
  }

  return CheckStaticCollision(rect);
}

bool Environment::CheckOptimizationCollision(double time, const math::Pose &pose, double collision_buffer) {
  math::AABox2d initial_box({-config_.vehicle.radius - collision_buffer, -config_.vehicle.radius - collision_buffer},
                            {config_.vehicle.radius + collision_buffer, config_.vehicle.radius + collision_buffer});

  double xr, yr, xf, yf;
  std::tie(xr, yr, xf, yf) = config_.vehicle.GetDiscPositions(pose.x(), pose.y(), pose.theta());

  auto f_box = initial_box, r_box = initial_box;
  f_box.Shift({xf, yf});
  r_box.Shift({xr, yr});
  if (CheckStaticCollision(math::Box2d(f_box)) || CheckStaticCollision(math::Box2d(r_box)) ||
      CheckDynamicCollision(time, math::Box2d(f_box)) ||
      CheckDynamicCollision(time, math::Box2d(r_box))) {
    return true;
  }
  return false;
}

bool Environment::CheckDynamicCollision(double time, const math::Box2d &rect) {
  for (auto &obstacle: dynamic_obstacles_) {
    if (obstacle.front().first > time || obstacle.back().first < time) {
      continue;
    }
    auto result = std::upper_bound(obstacle.begin(), obstacle.end(), time,
                                   [](double val, const std::pair<double, math::Polygon2d> &ob) {
                                     return val < ob.first;
                                   });

    if (result->second.HasOverlap(rect)) {
      return true;
    }
  }

  return false;
}

std::unordered_map<int, math::Polygon2d> Environment::QueryDynamicObstacles(double time) {
  std::unordered_map<int, math::Polygon2d> filtered;
  int idx = 0;
  for (auto &obstacle: dynamic_obstacles_) {
    idx++;
    if (obstacle.front().first > time || obstacle.back().first < time) {
      continue;
    }
    auto result = std::upper_bound(obstacle.begin(), obstacle.end(), time,
                                   [](double val, const std::pair<double, math::Polygon2d> &ob) {
                                     return val < ob.first;
                                   });

    filtered.insert({idx, result->second});
  }
  return filtered;
}

void Environment::Visualize() {
  std::vector<double> lb_x, lb_y, ub_x, ub_y;

  for (auto &point: reference_.data()) {
    auto lb = reference_.GetCartesian(point.s, point.left_bound);
    auto ub = reference_.GetCartesian(point.s, -point.right_bound);

    lb_x.push_back(lb.x());
    lb_y.push_back(lb.y());
    ub_x.push_back(ub.x());
    ub_y.push_back(ub.y());
  }

  visualization::Plot(lb_x, lb_y, 0.1, visualization::Color::Grey, 1, "Road Left");
  visualization::Plot(ub_x, ub_y, 0.1, visualization::Color::Grey, 1, "Road Right");

  int idx = 0;
  for (auto &obstacle: obstacles_) {
    visualization::PlotPolygon(obstacle, 0.1, visualization::Color::Magenta, idx++, "Obstacles");
  }

  // plot first frame of dynamic obstacles
  idx = 1;
  for (auto &obstacle: dynamic_obstacles_) {
    auto color = visualization::Color::fromHSV(int((double) idx / dynamic_obstacles_.size() * 320), 1.0, 1.0);
    color.set_alpha(0.5);
    visualization::PlotPolygon(obstacle[0].second, 0.1, color, idx, "Online Obstacle");
    idx++;
  }

  visualization::Trigger();
}

World::World(const float &resolution):resolution_(resolution)
{
    std::cout<<"class Word constuct done"<<std::endl;
    lowerbound_=INF*Vector3d::Ones();
    upperbound_=-INF*Vector3d::Ones();
    idx_count_=Vector3i::Zero();
}

World::~World(){clearMap();}

void World::clearMap()
{
    if(has_map_)
    {
        for(int i=0;i < idx_count_(0);i++)
        {
            for(int j=0;j < idx_count_(1);j++)
            {
                delete[] grid_map_[i][j];
                grid_map_[i][j]=NULL;
            }
            delete[] grid_map_[i];
            grid_map_[i]=NULL;
        }
        delete[] grid_map_;
        grid_map_=NULL;
    }
}

void World::initGridMap(const Vector3d &lowerbound,const Vector3d &upperbound)
{
    lowerbound_=lowerbound;
    upperbound_=upperbound;
    idx_count_=((upperbound_-lowerbound_)/resolution_).cast<int>()+Eigen::Vector3i::Ones();
    grid_map_=new bool**[idx_count_(0)];
    for(int i=0;i < idx_count_(0);i++)
    {
        grid_map_[i]=new bool*[idx_count_(1)];
        for(int j=0;j < idx_count_(1);j++)
        {
            grid_map_[i][j]=new bool[idx_count_(2)];
            memset(grid_map_[i][j],true,idx_count_(2)*sizeof(bool));
        }
    }
    has_map_=true;
}

void World::initGridMap(const pcl::PointCloud<pcl::PointXYZ> &cloud)  //need read
{   
    // std::cout<<"cloud.size()"<<cloud.size()<<std::endl;
    if(cloud.points.empty())
    {
        ROS_ERROR("Can not initialize the map with an empty point cloud!");
        return;
    }
    clearMap();

    for(const auto&pt:cloud.points)
    {
        if(pt.x < lowerbound_(0)) lowerbound_(0)=pt.x;
        if(pt.y < lowerbound_(1)) lowerbound_(1)=pt.y;
        if(pt.z < lowerbound_(2)) lowerbound_(2)=pt.z;
        if(pt.x > upperbound_(0)) upperbound_(0)=pt.x;
        if(pt.y > upperbound_(1)) upperbound_(1)=pt.y;
        if(pt.z + 1.0 > upperbound_(2)) upperbound_(2)=pt.z+1.0;
    }

    idx_count_ = ((upperbound_-lowerbound_)/resolution_).cast<int>() + Eigen::Vector3i::Ones();

    grid_map_=new bool**[idx_count_(0)];
    for(int i = 0 ; i < idx_count_(0) ; i++)
    {
        grid_map_[i]=new bool*[idx_count_(1)];
        for(int j = 0 ; j < idx_count_(1) ; j++)
        {
            grid_map_[i][j]=new bool[idx_count_(2)];
            memset(grid_map_[i][j],true,idx_count_(2)*sizeof(bool));
        }
    }
    has_map_=true;
}
 

void World::setObs(const Vector3d &point)
{   
    Vector3i idx=coord2index(point);
    grid_map_[idx(0)][idx(1)][idx(2)]=false;
}

bool World::isFree(const Vector3d &point)
{
    Vector3i idx = coord2index(point);
    bool is_free = isInsideBorder(idx) && grid_map_[idx(0)][idx(1)][idx(2)];  //有障碍物grid_map_为false
    return is_free;
}

Vector3d World::coordRounding(const Vector3d & coord)
{
    return index2coord(coord2index(coord));
}

bool World::project2surface(const float &x,const float &y,Vector3d* p_surface)
{
    bool ifsuccess=false;
    //  std::cout<<"in 0"<<std::endl;

    if(x>=lowerbound_(0) && x<=upperbound_(0) && y>=lowerbound_(1) && y<=upperbound_(1))
    {   
        // std::cout<<"in 1"<<std::endl;
        for(float z = lowerbound_(2) ; z < upperbound_(2) ; z+=resolution_)
        {
        // std::cout<<"in 2"<<std::endl;

            if( !isFree(x,y,z) && isFree(x,y,z+resolution_) )
            {
        // std::cout<<"in 3"<<std::endl;

                *p_surface=Vector3d(x,y,z);
                ifsuccess=true;
                break;
            }
        }
    }
    return ifsuccess;
}

bool World::CheckStaticCollision(const math::Box2d &rect) {
  // double xmax, ymax, xmin, ymin;
  double path_height = -0.3;

  double xmax = rect.center_x() + rect.half_length();
  double ymax = rect.center_y() + rect.half_width();
  double xmin = rect.center_x() - rect.half_length();
  double ymin = rect.center_y() - rect.half_width();

  int count_l = 0;
  int count_r = 0;
  double limt_min = path_height - 2*resolution_;
  double limt_max = path_height + resolution_;
  limt_min = max(lowerbound_(2),limt_min);
  limt_max = min(upperbound_(2),limt_max);

  for(double x = xmin ;x<=xmax ; x+=resolution_) 
  {     
        count_l++;
        count_r++;

        bool find_ground1 = false;
        bool find_ground2 = false;

        if(x<lowerbound_(0) || x>upperbound_(0) || ymin<lowerbound_(1) || ymin>upperbound_(1)|| ymax<lowerbound_(1) || ymax>upperbound_(1))
            return true;
        if(!isFree(x,ymin,path_height)||!isFree(x,ymax,path_height))
        {
            return true;
        }
        for(float z = limt_min ; z < limt_max ; z+=resolution_)
        {
              if(!isFree(x,ymin,z))
              {
                find_ground1 = true;
                count_l = 0;
              }
              if(!isFree(x,ymax,z))
              {
                find_ground2 = true;
                count_r = 0;
              }
              if(find_ground1&&find_ground2)  break;
        }
        //
        // if(!find_ground1||!find_ground2)  return true;
        //
        if(count_l==2||count_r==2)
        {
          return true;
        }
        if(x==xmax&&(count_l!=0||count_r!=0))
        {
          return true;
        }

        //******边缘***///
        if(x==xmin&&(count_l!=0||count_r!=0))
        {
          return true;
        }
  }

  int count_l_y = 0;
  int count_r_y = 0;
  for(double y = ymin ;y<=ymax ; y+=resolution_) 
  {     
        count_l_y++;
        count_r_y++;
        bool find_ground1 = false;
        bool find_ground2 = false;

        if(xmin<lowerbound_(0) || xmin>upperbound_(0)|| xmax<lowerbound_(0) || xmax>upperbound_(0) || y<lowerbound_(1) || y>upperbound_(1))
            return true;
        if(!isFree(xmin,y,path_height)||!isFree(xmax,y,path_height))
        {
            return true;
        }
        for(float z = limt_min ; z < limt_max ; z+=resolution_)
        {
              if(!isFree(xmin,y,z))
              {
                find_ground1 = true;
                count_l_y = 0;
              }
              if(!isFree(xmax,y,z))
              {
                find_ground2 = true;
                count_r_y = 0;

              }
              if(find_ground1&&find_ground2)  break;
        }
        //
        // if(!find_ground1||!find_ground2)  return true;
        //
        if(count_l_y==2||count_r_y==2)
        {
          return true;
        }
        if(y==ymax&&(count_l_y!=0||count_r_y!=0))
        {
          return true;
        }
  
        //******边缘***///
        if(y==ymin&&(count_l_y!=0||count_r_y!=0))
        {
          return true;
        }

  }
  // for(double x = xmin ;x<=xmax ; x+=getObstacle_map().info.resolution) 
  // {     
  //       int index_x = (int)round((x - getObstacle_map().info.origin.position.x) / getObstacle_map().info.resolution);
  //       int index_y_min = (int)round((ymin - getObstacle_map().info.origin.position.y) / getObstacle_map().info.resolution);
  //       int index_0 = index_x + index_y_min * getObstacle_map().info.width;
  //       if (index_x < 0 || index_x >= getObstacle_map().info.width || index_y_min < 0 || index_y_min >= getObstacle_map().info.height)    
  //            continue;
  //       if (getObstacle_map().data[index_0] != 0)
  //           return true;
  // }

  // for(double y = ymin ;y<=ymax ; y+=getObstacle_map().info.resolution)
  // {
  //       int index_x_min = (int)round((xmin - getObstacle_map().info.origin.position.x) / getObstacle_map().info.resolution);
  //       int index_y = (int)round((y - getObstacle_map().info.origin.position.y) / getObstacle_map().info.resolution);
  //       int index_0 = index_x_min + index_y * getObstacle_map().info.width;
  //       if (index_x_min < 0 || index_x_min >= getObstacle_map().info.width || 
  //           index_y < 0 || index_y >= getObstacle_map().info.height)
  //           continue;
  //       if (getObstacle_map().data[index_0] != 0)
  //           return true;
  // }
  // for(double x = xmin ;x<=xmax ; x+=getObstacle_map().info.resolution) 
  // {     
  //       int index_x = (int)round((x - getObstacle_map().info.origin.position.x) / getObstacle_map().info.resolution);
  //       int index_y_max = (int)round((ymax - getObstacle_map().info.origin.position.y) / getObstacle_map().info.resolution);
  //       int index_1 = index_x + index_y_max * getObstacle_map().info.width;
  //       if (index_x < 0 || index_x >= getObstacle_map().info.width ||
  //           index_y_max < 0 || index_y_max >= getObstacle_map().info.height)    
  //           continue;
  //       if (getObstacle_map().data[index_1] != 0)
  //           return true;
  // }

  // for(double y = ymin ;y<=ymax ; y+=getObstacle_map().info.resolution)
  // {
  //       int index_x_max = (int)round((xmax - getObstacle_map().info.origin.position.x) / getObstacle_map().info.resolution);
  //       int index_y = (int)round((y - getObstacle_map().info.origin.position.y) / getObstacle_map().info.resolution);
  //       int index_1 = index_x_max + index_y * getObstacle_map().info.width;
  //       if ( index_x_max < 0 || index_x_max >= getObstacle_map().info.width||
  //           index_y < 0 || index_y >= getObstacle_map().info.height)
  //           continue;
  //       if (getObstacle_map().data[index_1] != 0)
  //           return true;
  // }

  return false;
}

bool World::isInsideBorder(const Vector3i &index)
{
    return index(0) >= 0 &&
           index(1) >= 0 &&
           index(2) >= 0 && 
           index(0) < idx_count_(0)&&
           index(1) < idx_count_(1)&&
           index(2) < idx_count_(2);
}

void World::visWorld( ros::Publisher* world_vis_pub)
{
  if (world_vis_pub == NULL || !has_map_)
    return;
  pcl::PointCloud<pcl::PointXYZ> cloud_vis;
  for (int i = 0; i < idx_count_(0); i++)
  {
    for (int j = 0; j < idx_count_(1); j++)
    {
      for (int k = 0; k < idx_count_(2); k++)
      {
        Vector3i index(i, j, k);
        if (!grid_map_[index(0)][index(1)][index(2)])
        {
          Vector3d coor_round = index2coord(index);
          pcl::PointXYZ pt_add;
          pt_add.x = coor_round(0);
          pt_add.y = coor_round(1);
          pt_add.z = coor_round(2);
          cloud_vis.points.push_back(pt_add);
        }
      }
    }
  }

  cloud_vis.width = cloud_vis.points.size();
  cloud_vis.height = 1;
  cloud_vis.is_dense = true;

  sensor_msgs::PointCloud2 map_vis;
  pcl::toROSMsg(cloud_vis, map_vis);

  map_vis.header.frame_id = "/map";
  world_vis_pub->publish(map_vis);
}

}
