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

#pragma once

#include <memory>
#include <unordered_map>

#include "math/polygon2d.h"
#include "discretized_trajectory.h"
#include "vehicle_param.h"
#include "cartesian_planner_config.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

#include <tf/tf.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>


#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/filters/filter.h>
#include <pcl_ros/filters/voxel_grid.h>
#include <pcl_ros/filters/passthrough.h>
#include <pcl_ros/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
// #include <utility>


namespace cartesian_planner {

class Environment {
public:
  using DynamicObstacle = std::vector<std::pair<double, math::Polygon2d>>;

  explicit Environment(const CartesianPlannerConfig &config) : config_(config) {}

  std::vector<math::Polygon2d> &obstacles() {
    return obstacles_;
  }

  std::vector<DynamicObstacle> &dynamic_obstacles() {
    return dynamic_obstacles_;
  }

  const DiscretizedTrajectory &reference() const {
    return reference_;
  }

  const nav_msgs::Path &getreference_path() const {
    return reference_path_;
  }

  const nav_msgs::OccupancyGrid &getObstacle_map() const {
    return obstacles_map_;
  }

  void SetReference(const DiscretizedTrajectory &reference);
  void SetReference(const nav_msgs::Path &reference_path);
  void SetObstacles_map(const nav_msgs::OccupancyGrid &obstacles_map);




  bool CheckCollision(double time, const math::Box2d &rect);

  bool CheckOptimizationCollision(double time, const math::Pose &pose, double collision_buffer = 0.0);

  std::unordered_map<int, math::Polygon2d> QueryDynamicObstacles(double time);

  void Visualize();

private:
  CartesianPlannerConfig config_;
  std::vector<DynamicObstacle> dynamic_obstacles_;
  std::vector<math::Polygon2d> obstacles_;
  DiscretizedTrajectory reference_;
  std::vector<math::Vec2d> road_barrier_;
  nav_msgs::Path reference_path_;
  nav_msgs::OccupancyGrid obstacles_map_;


  bool CheckStaticCollision(const math::Box2d &rect);

  bool CheckDynamicCollision(double time, const math::Box2d &rect);
};


const float INF= std::numeric_limits<float>::max();
const float PI = 3.14151f;

class World
{
public:
    // friend void visualization::visWorld(World* world,ros::Publisher* world_vis_pub);

    //indicate whether the range of the grid map has been determined
    bool has_map_=false;

    World(const float &resolution);
    ~World();

    /**
     * @brief Automatically determine the upperbound and lowerbound of the grid map according to the
     *        information of the input point cloud.
     * @param pcl::PointCloud<pcl::PointXYZ> point cloud input
     * @return void
     */
    void initGridMap(const pcl::PointCloud<pcl::PointXYZ> &cloud);

    /**
     * @brief Manually specify the upperbound and lowerbound.
     * @param Vector3d
     * @param Vector3d
     * @return void
     */
    void initGridMap(const Eigen::Vector3d &lowerbound,const Eigen::Vector3d &upperbound);
    void setObs(const Eigen::Vector3d &point);

    /**
     * @brief Find the grid closet to the point and return the coordinate of its center
     * @param Vector3d
     * @return Vector3d
     */
    Eigen::Vector3d coordRounding(const Eigen::Vector3d &coord);

    bool isFree(const Eigen::Vector3d &point);
    bool isFree(const float &coord_x, const float &coord_y, const float &coord_z){return isFree(Eigen::Vector3d(coord_x,coord_y,coord_z));}

    /**
     * @brief Given a 2D coord,start from the lowerbound of the height of the grid map,search upward,
     *        and determine the boundary between the occupied area and the non occupied area as the 
     *        surface point.
     * @param float x(the first dimension)
     * @param float y(the second dimension)
     * @param Vector3d* p_surface(store the result of the projecting)
     * @return bool true(no obstacle exists),false(exist obstacle)
     */
    bool project2surface(const float &x,const float &y,Eigen::Vector3d* p_surface); 
    bool project2surface(const Eigen::Vector3d &p_original,Eigen::Vector3d* p_surface){return project2surface(p_original(0),p_original(1),p_surface);}

     /**
     * @brief Check if there is any obstacle between 2 nodes.
     * @param Node* node_start
     * @param Node* node_end
     * @return bool true(no obstacle exists),false(exist obstacle)
     */
    
    /**
     * @brief Check whether the given point is within the range of the grid map
     * @param Eigen::Vector3i(the index value obtained after discretization of the given point)
     * @return bool true(within range),false（out of range)
     */
    bool isInsideBorder(const Eigen::Vector3i &index);
    bool isInsideBorder(const Eigen::Vector3d &point){return isInsideBorder(coord2index(point));}

    /**
     * @brief get the low bound of the world
     * @param void
     * @return Vector3d
     */
    Eigen::Vector3d getLowerBound(){return lowerbound_;}

    /**
     * @brief get the up bound of the world
     * @param void
     * @return Vector3d
     */
    Eigen::Vector3d getUpperBound(){return upperbound_;}

    /**
     * @brief get resolution of the world
     * @param void
     * @return float
     */
    float getResolution(){return resolution_;} 

    Eigen::Vector3d index2coord(const Eigen::Vector3i &index)
    {
        Eigen::Vector3d coord = resolution_*index.cast<double>() + lowerbound_+ 0.5*resolution_*Eigen::Vector3d::Ones();
        return coord;
    }

    Eigen::Vector3i coord2index(const Eigen::Vector3d &coord)
    {
        Eigen::Vector3i index = ( (coord-lowerbound_)/resolution_).cast<int>();            
        return index;
    }
//protected:
    bool ***grid_map_=NULL;

    float resolution_;

    Eigen::Vector3i idx_count_;

    Eigen::Vector3d lowerbound_;
    Eigen::Vector3d upperbound_;

    void clearMap();
};


using Env = std::shared_ptr<Environment>;
}
