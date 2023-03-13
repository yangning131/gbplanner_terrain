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


#include "geometry_msgs/PoseStamped.h"
#include "cartesian_planner/CenterLine.h"
#include "cartesian_planner/Obstacles.h"
#include "cartesian_planner/DynamicObstacles.h"
#include "cartesian_planner/cartesian_planner.h"
#include "cartesian_planner/Polynome.h"

#include "cartesian_planner/visualization/plot.h"




using namespace cartesian_planner;

class CartesianPlannerNode {
public:
  std::mutex mtx;
  nav_msgs::Path globalPathMessage;
  CartesianPlanner::StartState robotstate;
  tf::TransformListener listener;
  tf::StampedTransform transform;
  bool receive = false;
  bool reach = false;

  ros::Timer pathUpdateTimer;
  ros::Publisher path_pub_;
  ros::Publisher traj_pub;



  explicit CartesianPlannerNode(const ros::NodeHandle &nh) : nh_(nh) {
    env_ = std::make_shared<Environment>(config_);
    world_ = std::make_shared<World>(0.2, 0.2);

    planner_ = std::make_shared<CartesianPlanner>(config_, env_, world_);
    
    obstacles_subscriber_ = nh_.subscribe("/obstacles11", 1, &CartesianPlannerNode::ObstaclesCallback, this);
    dynamic_obstacles_subscriber_ = nh_.subscribe("/dynamic_obstacles11", 1,
                                                  &CartesianPlannerNode::DynamicObstaclesCallback, this);//每个时间点对应障碍物的坐标

    or_path_subscriber_ = nh_.subscribe("expath222", 1, &CartesianPlannerNode::Pathcallback, this); //planning/planning/execute_path  planning/server/path_blueprint_smooth
    subObstacleMap_ = nh_.subscribe<nav_msgs::OccupancyGrid>("planning/obstacle/map_inflated", 5, &CartesianPlannerNode::mapHandler, this);
    
    cloud_terrain_sub = nh_.subscribe<sensor_msgs::PointCloud2>("/cloud_registered", 1,&CartesianPlannerNode::cloudHandler2,this);//cloud_registered   laser_cloud_map

    pathUpdateTimer = nh_.createTimer(ros::Duration(0.3), &CartesianPlannerNode::updatePath, this);
    path_pub_ = nh_.advertise<nav_msgs::Path>("planning/planning/execute_path_op", 1);
    grid_map_vis_pub = nh_.advertise<sensor_msgs::PointCloud2>("grid_map_vis_carte", 1);
    H_grid_map_vis_pub = nh_.advertise<sensor_msgs::PointCloud2>("H_grid_map_vis_carte", 1);

    traj_pub      = nh_.advertise<cartesian_planner::Polynome>("trajectory",3);

  }



  
  inline float NormalizeAngle_pi(const float angle) const
  {
        double v = fmod(angle, 2 * M_PI);  //0~pi
  
          if (v < -M_PI) {
           v += 2.0 * M_PI;
      } else if (v > M_PI) {
         v -= 2.0 * M_PI;
      }

     return v;
  }

  double Mod2Pi(const double &x)  {       
    double v = fmod(x, 2 * M_PI);
  
      if (v < -M_PI) {
        v += 2.0 * M_PI;
    } else if (v > M_PI) {
        v -= 2.0 * M_PI;
    }
    return v;
}

  bool getRobotPosition()
  {
        try{listener.lookupTransform("map","base_link", ros::Time(0), transform); } 
        catch (tf::TransformException ex){ /*ROS_ERROR("Transfrom Failure.");*/ return false; }
        
        state_.x = transform.getOrigin().x();
        state_.y = transform.getOrigin().y();
        state_.z = transform.getOrigin().z();

        double roll, pitch, yaw;
        tf::Matrix3x3 m(transform.getRotation());
        m.getRPY(roll, pitch, yaw);
        state_.theta =  Mod2Pi(yaw);


    state_.v = 0.3;
    state_.phi = 0.0;
    state_.a = 0.0;
    state_.omega = 0.0;

        return true;
  }

  void CenterLineCallback(const CenterLineConstPtr &msg) {
    Trajectory data;
    for (auto &pt: msg->points) {
      TrajectoryPoint tp;
      tp.s = pt.s;
      tp.x = pt.x;
      tp.y = pt.y;
      tp.theta = pt.theta;
      tp.kappa = pt.kappa;
      tp.left_bound = pt.left_bound;
      tp.right_bound = pt.right_bound;
      data.push_back(tp);
    }

    env_->SetReference(DiscretizedTrajectory(data));
    env_->Visualize();
  }

  void mapHandler(const nav_msgs::OccupancyGrid::ConstPtr& mapMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);
        nav_msgs::OccupancyGrid occupancyMap2D;
        occupancyMap2D = *mapMsg;
        env_->SetObstacles_map(occupancyMap2D);
    }
  float pointDistance(pcl::PointXYZ p1, pcl::PointXYZ p2)
  {
        return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
  }

  void cloudHandler2(const sensor_msgs::PointCloud2::ConstPtr& laserCloudMsg)
    {    
        std::lock_guard<std::mutex> lock(mtx);
        double timeScanCur = laserCloudMsg->header.stamp.toSec();

        sensor_msgs::PointCloud2 pointcloud_map;
       
        pointcloud_map = *laserCloudMsg;

        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(pointcloud_map, cloud);

        pcl::PointCloud<pcl::PointXYZ> Localcloud;
        pcl::PointXYZ robot_point;
        robot_point.x = state_.x;
        robot_point.y = state_.y;

        // clock_t start,end;
        // start=clock();
        for (int i = 0; i < cloud.size(); ++i)
        {
            pcl::PointXYZ p;
            p.x = cloud.points[i].x;
            p.y = cloud.points[i].y;
            p.z = cloud.points[i].z;
            // if (p.z > _sensorHeightLimitUpper + robotPoint.z || p.z < _sensorHeightLimitDown + robotPoint.z)
            //     continue;

            float range = pointDistance(p, robot_point);
            if (range < 0 || range > 10.0)
                continue;
            Localcloud.push_back(p);
        }
            // end=clock();
            // double endtime=(double)(end-start)/CLOCKS_PER_SEC;
            // std::cout<<"Total time0:"<<endtime*1000<<"ms"<<std::endl;

        cloudQueue.push_back(Localcloud);
        timeQueue.push_back(timeScanCur);

        while (!timeQueue.empty())
        {
          if(timeScanCur - timeQueue.front()> 20.0)
          {
            cloudQueue.pop_front();
            timeQueue.pop_front();
          }else{
            break;
          }
        }
        pcl::PointCloud<pcl::PointXYZ> surroundMapCloud;
        for(int i = 0 ;i<cloudQueue.size();i++)
            surroundMapCloud += cloudQueue[i];
            // start=clock();

        world_->initGridMap(surroundMapCloud);


        for (const auto& pt : surroundMapCloud)
        {
          Eigen::Vector3d obstacle(pt.x, pt.y, pt.z);
          world_->setObs(obstacle);//x ,y ,z 三个index
        }
            //                 end=clock();
            //  endtime=(double)(end-start)/CLOCKS_PER_SEC;
            // std::cout<<"Total time1:"<<endtime*1000<<"ms"<<std::endl;
        world_->visWorld( &grid_map_vis_pub);
        world_->H_visWorld( &H_grid_map_vis_pub);

    }
  void ObstaclesCallback(const ObstaclesConstPtr &msg) {
    env_->obstacles().clear();
    for (auto &obstacle: msg->obstacles) {
      std::vector<math::Vec2d> points;
      for (auto &pt: obstacle.points) {
        points.emplace_back(pt.x, pt.y);
      }
      env_->obstacles().emplace_back(points);
    }
    env_->Visualize();
  }

  void DynamicObstaclesCallback(const DynamicObstaclesConstPtr &msg) {
    env_->dynamic_obstacles().clear();
    for (auto &obstacle: msg->obstacles) {
      Environment::DynamicObstacle dynamic_obstacle;

      for (auto &tp: obstacle.trajectory) {
        math::Pose coord(tp.x, tp.y, tp.theta);
        std::vector<math::Vec2d> points;
        for (auto &pt: obstacle.polygon.points) {
          points.push_back(coord.transform({pt.x, pt.y, 0.0}));
        }
        math::Polygon2d polygon(points);

        dynamic_obstacle.emplace_back(tp.time, points);
      }

      env_->dynamic_obstacles().push_back(dynamic_obstacle);
    }
    env_->Visualize();
  }


double cast_from_0_to_2PI_Angle(const double& ang)
{
    double angle = 0;
    if (ang < -2.0 * M_PI || ang > 2.0 * M_PI) {
        angle = fmod(ang, 2.0 * M_PI);
    } else
        angle = ang;

    if (angle < 0) {
        angle = 2.0 * M_PI + angle;
    }
    return angle;
}

  void Pathcallback(const nav_msgs::Path::ConstPtr& pathMsg) {
      std::lock_guard<std::mutex> lock(mtx); 
      if (pathMsg->poses.size() <= 1)
      {
          ROS_WARN("Empty global path received.");
          return;
      }
      std::cout<<"receive path size"<<pathMsg->poses.size()<<std::endl;
      globalPathMessage = *pathMsg;
      env_->SetReference(globalPathMessage);
      receive = true;
  }




  void updatePath(const ros::TimerEvent& event)
  {
    std::lock_guard<std::mutex> lock(mtx);

    if (getRobotPosition() == false) return;
    


    DiscretizedTrajectory result;
    if(receive||reach)
    {
    if (planner_->Plan(state_, result)) {
        nav_msgs::Path nav_path;
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "map";

      for (int i = 0; i < config_.nfe; i++) {
        auto &pt = result.data().at(i);
        pose_stamped.pose.position.x = pt.x;
        pose_stamped.pose.position.y = pt.y;
        pose_stamped.pose.position.z = pt.z;
        pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(pt.theta);

        nav_path.poses.emplace_back(pose_stamped);
      }
      end_pose = pose_stamped;
      nav_path.header.frame_id = "map";
      nav_path.header.stamp = ros::Time::now();
      last_path = nav_path;
      path_pub_.publish(nav_path);


      cartesian_planner::Polynome poly;
      for (int i = 0; i < config_.nfe; i++) {
        geometry_msgs::Point temp;

        auto &pt = result.data().at(i);
        temp.x = pt.x;
        temp.y = pt.y;
        temp.z = pt.z;
        poly.pos_pts.push_back(temp);
        poly.t_pts.push_back(0.05);
      }
      poly.init_v.x = 0;
      poly.init_v.y = 0;
      poly.init_v.z = 0;
      poly.init_a.x = 0;
      poly.init_a.y = 0;
      poly.init_a.z = 0;
      poly.start_time = ros::Time::now();
      traj_pub.publish(poly);

      
      // double dt = config_.tf / (double) (config_.nfe - 1);  reach
      // for (int i = 0; i < config_.nfe; i++) {
      //   double time = dt * i;
      //   auto dynamic_obstacles = env_->QueryDynamicObstacles(time);
      //   for (auto &obstacle: dynamic_obstacles) {
      //     int hue = int((double) obstacle.first / env_->dynamic_obstacles().size() * 320);

      //     visualization::PlotPolygon(obstacle.second, 0.2, visualization::Color::fromHSV(hue, 1.0, 1.0), obstacle.first,
      //                                "Online Obstacle");
      //   }

      //   auto &pt = result.data().at(i);
      //   PlotVehicle(1, {pt.x, pt.y, pt.theta}, atan(pt.kappa * config_.vehicle.wheel_base));
      //   ros::Duration(dt).sleep();
      // }

      // visualization::Trigger();
    }
      receive = false;
      reach = false;
    }
    if(0.4>hypot(hypot(end_pose.pose.position.x - state_.x, end_pose.pose.position.y - state_.y ) ,
                   end_pose.pose.position.z - state_.z) )    reach = true;





  }

private:
  ros::NodeHandle nh_;
  cartesian_planner::CartesianPlannerConfig config_;
  Env env_;
  // cartesian_planner::World* world_;
  Wor world_;
  std::shared_ptr<cartesian_planner::CartesianPlanner> planner_;
  CartesianPlanner::StartState state_;

  ros::Subscriber  obstacles_subscriber_, dynamic_obstacles_subscriber_, goal_subscriber_, subObstacleMap_, cloud_terrain_sub, or_path_subscriber_;

  ros::Publisher grid_map_vis_pub;
  ros::Publisher H_grid_map_vis_pub;
  nav_msgs::Path last_path;
  std::deque<pcl::PointCloud<pcl::PointXYZ>> cloudQueue;
  std::deque<double> timeQueue;
  geometry_msgs::PoseStamped  end_pose;

  void PlotVehicle(int id, const math::Pose &pt, double phi) {
    auto tires = GenerateTireBoxes({pt.x(), pt.y(), pt.theta()}, phi);

    int tire_id = 1;
    for (auto &tire: tires) {
      visualization::PlotPolygon(math::Polygon2d(tire), 0.1, visualization::Color::White, id * (tire_id++),
                                 "Tires");
    }
    visualization::PlotPolygon(math::Polygon2d(config_.vehicle.GenerateBox({pt.x(), pt.y(), pt.theta()})), 0.2,
                               visualization::Color::Yellow, id, "Footprint");
    visualization::Trigger();
  }

  std::array<math::Box2d, 4> GenerateTireBoxes(const math::Pose pose, double phi = 0.0) const {
    auto front_pose = pose.extend(config_.vehicle.wheel_base);
    auto track_width = config_.vehicle.width - 0.195;
    double rear_track_width_2 = track_width / 2, front_track_width_2 = track_width / 2;
    double box_length = 0.6345;
    double sin_t = sin(pose.theta());
    double cos_t = cos(pose.theta());
    return {
      math::Box2d({pose.x() - rear_track_width_2 * sin_t, pose.y() + rear_track_width_2 * cos_t}, pose.theta(),
                  box_length, 0.195),
      math::Box2d({pose.x() + rear_track_width_2 * sin_t, pose.y() - rear_track_width_2 * cos_t}, pose.theta(),
                  box_length, 0.195),
      math::Box2d({front_pose.x() - front_track_width_2 * sin_t, front_pose.y() + front_track_width_2 * cos_t},
                  front_pose.theta() + phi, box_length, 0.195),
      math::Box2d({front_pose.x() + front_track_width_2 * sin_t, front_pose.y() - front_track_width_2 * cos_t},
                  front_pose.theta() + phi, box_length, 0.195),
    };
  }
};
// 35 -15
int main(int argc, char **argv) {
  ros::init(argc, argv, "cartesian_planner_node");

  ros::NodeHandle nh;
  visualization::Init(nh, "map", "cartesian_planner_markers");

  CartesianPlannerNode node(nh);
  ros::spin();
  return 0;
}