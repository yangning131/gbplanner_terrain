/* A ROS implementation of the Pure pursuit path tracking algorithm (Coulter 1992).

   Terminology (mostly :) follows:
   Coulter, Implementation of the pure pursuit algoritm, 1992 and 
   Sorniotti et al. Path tracking for Automated Driving, 2017.
 */

#include <string>
#include <cmath>
#include <algorithm>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <tf/tf.h>

#include <kdl/frames.hpp>

// TODO: Figure out how to use tf2 DataConversions
// for more elegant and compact code
//#include <tf2_kdl/tf2_kdl.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <dynamic_reconfigure/server.h>
#include <pure_p/PurePursuitConfig.h>

using std::string;

class PurePursuit
{
public:

  //! Constructor
  PurePursuit();

  //! Compute velocit commands each time new odometry data is received.
  void computeVelocities(nav_msgs::Odometry odom);
  double pointdistance(double x1,double x2,double y1,double y2)
  {
    return sqrt(pow(x1 - x2,2) + pow(y1 - y2,2) );
  }

  double cast_from_0_to_2PI_Angle(const double& ang)//余数  化为-2pi到2pi的数
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

  //! Receive path to follow.
  void receivePath(nav_msgs::Path path);

  //! Compute transform that transforms a pose into the robot frame (base_link)
  KDL::Frame transformToBaseLink(const geometry_msgs::Pose& pose,
                                 const geometry_msgs::Transform& tf);
  
  //! Helper founction for computing eucledian distances in the x-y plane.
  template<typename T1, typename T2>
  double distance(T1 pt1, T2 pt2)
  {
    return sqrt(pow(pt1.x - pt2.x,2) + pow(pt1.y - pt2.y,2) + pow(pt1.z - pt2.z,2));
  }

  //! Run the controller.
  void run();
  
private:

  //! Dynamic reconfigure callback.
  void reconfigure(pure_pursuit::PurePursuitConfig &config, uint32_t level);
  
  // Vehicle parameters
  double L_;
  // Algorithm variables
  // Position tolerace is measured along the x-axis of the robot!
  double ld_, pos_tol_;
  // Generic control variables
  double v_max_, v_, w_max_;
  // Control variables for Ackermann steering
  // Steering angle is denoted by delta
  double delta_, delta_vel_, acc_, jerk_, delta_max_;
  nav_msgs::Path path_;
  nav_msgs::Path path_show;
  nav_msgs::Path path_p;



  nav_msgs::Path  mypath;

  unsigned idx_;
  bool goal_reached_;
  geometry_msgs::Twist cmd_vel_;
  ackermann_msgs::AckermannDriveStamped cmd_acker_;
  
  // Ros infrastructure
  ros::NodeHandle nh_, nh_private_;
  ros::Subscriber sub_odom_, sub_path_;
  ros::Publisher pub_vel_, pub_acker_;
  ros::Publisher pub_path;
  ros::Publisher pub_path_show;


  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  geometry_msgs::TransformStamped lookahead_;
  string map_frame_id_, robot_frame_id_, lookahead_frame_id_, acker_frame_id_;

  dynamic_reconfigure::Server<pure_pursuit::PurePursuitConfig> reconfigure_server_;
  dynamic_reconfigure::Server<pure_pursuit::PurePursuitConfig>::CallbackType reconfigure_callback_;
  
};

PurePursuit::PurePursuit() : ld_(1.0), v_max_(0.3), v_(v_max_), w_max_(0.5), pos_tol_(0.1), idx_(0),
                             goal_reached_(false), nh_private_("~"), tf_listener_(tf_buffer_),
                             map_frame_id_("camera_init"), robot_frame_id_("odom_imu"),
                             lookahead_frame_id_("lookahead2")
{
  // Get parameters from the parameter server
  nh_private_.param<double>("wheelbase", L_, 1.0);
  nh_private_.param<double>("lookahead_distance", ld_, 1.0);
  nh_private_.param<double>("linear_velocity", v_, 0.3);
  nh_private_.param<double>("max_rotational_velocity", w_max_, 0.5);
  nh_private_.param<double>("position_tolerance", pos_tol_, 0.1);
  nh_private_.param<double>("steering_angle_velocity", delta_vel_, 100.0);
  nh_private_.param<double>("acceleration", acc_, 100.0);
  nh_private_.param<double>("jerk", jerk_, 100.0);
  nh_private_.param<double>("steering_angle_limit", delta_max_, 1.57);
  nh_private_.param<string>("map_frame_id", map_frame_id_, "camera_init");
  // Frame attached to midpoint of rear axle (for front-steered vehicles).
  nh_private_.param<string>("robot_frame_id", robot_frame_id_, "odom_imu");
  // Lookahead frame moving along the path as the vehicle is moving.
  nh_private_.param<string>("lookahead_frame_id", lookahead_frame_id_, "lookahead2");
  // Frame attached to midpoint of front axle (for front-steered vehicles).
  nh_private_.param<string>("ackermann_frame_id", acker_frame_id_, "rear_axle_midpoint");

  // Populate messages with static data
  lookahead_.header.frame_id = robot_frame_id_;
  lookahead_.child_frame_id = lookahead_frame_id_;

  cmd_acker_.header.frame_id = acker_frame_id_;
  cmd_acker_.drive.steering_angle_velocity = delta_vel_;
  cmd_acker_.drive.acceleration = acc_;
  cmd_acker_.drive.jerk = jerk_;
  
  sub_path_ = nh_.subscribe("pathsmooth", 1, &PurePursuit::receivePath, this);
  sub_odom_ = nh_.subscribe("odom", 1, &PurePursuit::computeVelocities, this);
  // pub_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  pub_path = nh_.advertise<nav_msgs::Path>("mypath", 1);
  pub_path_show = nh_.advertise<nav_msgs::Path>("mypath_show", 1);

  // pub_acker_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("cmd_acker", 1);

  reconfigure_callback_ = boost::bind(&PurePursuit::reconfigure, this, _1, _2);
  reconfigure_server_.setCallback(reconfigure_callback_);
}

void PurePursuit::computeVelocities(nav_msgs::Odometry odom)
{
  // The velocity commands are computed, each time a new Odometry message is received.
  // Odometry is not used directly, but through the tf tree.

  // Get the current robot pose
  // geometry_msgs::TransformStamped tf;
  try
  {
     
    path_.header.frame_id="map";
    path_.header.stamp=ros::Time::now();

    path_show.header.frame_id="map";
    path_show.header.stamp=ros::Time::now();

    path_p.header.frame_id="map";
    path_p.header.stamp=ros::Time::now();

     if (goal_reached_)
     {    
          float min_dist = FLT_MAX;
          int min_id = -1;
          for(int i  =0 ;i< path_.poses.size();++i)
          {
            double xpath = path_.poses[i].pose.position.x;
            double ypath = path_.poses[i].pose.position.y;
            double dist  = pointdistance(xpath,odom.pose.pose.position.x ,ypath,odom.pose.pose.position.y );
            if(dist < min_dist)
            {
              min_dist = dist;
              min_id = i;

            }
          }

           if (min_id >= 0 && min_dist < 0.8)
            path_.poses.erase(path_.poses.begin(), path_.poses.begin() + min_id);


            path_p = path_;
            for(int i  =0 ;i< path_.poses.size();++i)
            { 
              path_p.poses[i].pose.position.z = path_.poses[i].pose.position.z - 0.4;//change
              // path_show.poses[i].pose.position.z = 0.0;
            }


          pub_path.publish(path_p);


          path_show = path_;
            for(int i  =0 ;i< path_.poses.size();++i)
            { 
              // path_show.poses[i].pose.position.z = 0.8;//change
              path_show.poses[i].pose.position.z = path_.poses[i].pose.position.z+0.8;
            }
                       

          pub_path_show.publish(path_show);

     }



  //  ros::Time now = ros::Time::now();
   
  //   tf = tf_buffer_.lookupTransform(map_frame_id_, robot_frame_id_, ros::Time(0), ros::Duration(10));
  //   // We first compute the new point to track, based on our current pose,
  //   // path information and lookahead distance.
  //   for (; idx_ < path_.poses.size(); idx_++)
  //   {
  //     if (distance(path_.poses[idx_].pose.position, tf.transform.translation) > ld_)
  //     {

  //       // Transformed lookahead to base_link frame is lateral error
  //       KDL::Frame F_bl_ld = transformToBaseLink(path_.poses[idx_].pose, tf.transform);
  //       lookahead_.transform.translation.x = F_bl_ld.p.x();
  //       lookahead_.transform.translation.y = F_bl_ld.p.y();
  //       lookahead_.transform.translation.z = F_bl_ld.p.z();
  //       F_bl_ld.M.GetQuaternion(lookahead_.transform.rotation.x,
  //                               lookahead_.transform.rotation.y,
  //                               lookahead_.transform.rotation.z,
  //                               lookahead_.transform.rotation.w);
        
  //       // TODO: See how the above conversion can be done more elegantly
  //       // using tf2_kdl and tf2_geometry_msgs

  //       break;
  //     }
  //   }

  //   if (!path_.poses.empty() && idx_ >= path_.poses.size())
  //   {
  //     // We are approaching the goal,
  //     // which is closer than ld

  //     // This is the pose of the goal w.r.t. the base_link frame
  //     KDL::Frame F_bl_end = transformToBaseLink(path_.poses.back().pose, tf.transform);

  //     if (fabs(F_bl_end.p.x()) <= pos_tol_)
  //     {
  //       // We have reached the goal
  //       goal_reached_ = true;

  //       // Reset the path
  //       path_ = nav_msgs::Path();
  //     }
  //     else
  //     {
  //       // We need to extend the lookahead distance
  //       // beyond the goal point.
      
  //       // Find the intersection between the circle of radius ld
  //       // centered at the robot (origin)
  //       // and the line defined by the last path pose
  //       double roll, pitch, yaw;
  //       F_bl_end.M.GetRPY(roll, pitch, yaw);



        
  //       double k_end = tan(yaw); // Slope of line defined by the last path pose
  //       double l_end = F_bl_end.p.y() - k_end * F_bl_end.p.x();
  //       double a = 1 + k_end * k_end;
  //       double b = 2 * l_end;
  //       double c = l_end * l_end - ld_ * ld_;
  //       double D = sqrt(b*b - 4*a*c);
  //       double x_ld = (-b + copysign(D,v_)) / (2*a);
  //       double y_ld = k_end * x_ld + l_end;
        
  //       lookahead_.transform.translation.x = x_ld;
  //       lookahead_.transform.translation.y = y_ld;
  //       lookahead_.transform.translation.z = F_bl_end.p.z();
  //       F_bl_end.M.GetQuaternion(lookahead_.transform.rotation.x,
  //                                lookahead_.transform.rotation.y,
  //                                lookahead_.transform.rotation.z,
  //                                lookahead_.transform.rotation.w);
  //     }
  //   }

  //   if (!goal_reached_)
  //   {
  //     // We are tracking.

  //     // Compute linear velocity.
 
  //     // Right now,this is not very smart :)
  //     v_ = copysign(v_max_, v_);
      
    
  //     // Compute the angular velocity.
  //     // Lateral error is the y-value of the lookahead point (in base_link frame)
  //     double yt = lookahead_.transform.translation.y;
  //     double ld_2 = ld_ * ld_;

  //     double an_z;
  //       // ROS_INFO("w_max_ %f",w_max_);
  //        ROS_INFO("ld_ %f",ld_);

  //       an_z = 2*v_ / ld_2 * yt;
  //     if (2*v_ / ld_2 * yt<-2*w_max_)
  //     {
  //       /* code */

  //       an_z =  -2*w_max_;
  //     }
  //     if (2*v_ / ld_2 * yt>w_max_)
  //     {
  //       /* code */

  //       an_z =  w_max_;
  //     }

      


  //     // cmd_vel_.angular.z = std::min( 2*v_ / ld_2 * yt, w_max_ );
  //      cmd_vel_.angular.z = an_z;

  //     // Compute desired Ackermann steering angle
  //     cmd_acker_.drive.steering_angle = std::min( atan2(2 * yt * L_, ld_2), delta_max_ );
      
  //     // Set linear velocity for tracking.
  //     cmd_vel_.linear.x = v_;
  //     cmd_acker_.drive.speed = v_;

  //     cmd_acker_.header.stamp = ros::Time::now();



    // }
    // else
    // {
      // We are at the goal!

      // Stop the vehicle
      
      // The lookahead target is at our current pose.
      // lookahead_.transform = geometry_msgs::Transform();
      // lookahead_.transform.rotation.w = 1.0;
      
      // // Stop moving.
      // cmd_vel_.linear.x = 0.0;
      // cmd_vel_.angular.z = 0.0;

      // cmd_acker_.header.stamp = ros::Time::now();
      // cmd_acker_.drive.steering_angle = 0.0;
      // cmd_acker_.drive.speed = 0.0;
    // }

    // Publish the lookahead target transform.
    // lookahead_.header.stamp = ros::Time::now();
    // tf_broadcaster_.sendTransform(lookahead_);
    
    // Publish the velocities
    // pub_vel_.publish(cmd_vel_);
    
    // Publish ackerman steering setpoints
    // pub_acker_.publish(cmd_acker_);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN_STREAM(ex.what());
  }
}

void PurePursuit::receivePath(nav_msgs::Path new_path)
{
  // When a new path received, the previous one is simply discarded
  // It is up to the planner/motion manager to make sure that the new
  // path is feasible.
  // Callbacks are non-interruptible, so this will
  // not interfere with velocity computation callback.
  
  if (new_path.header.frame_id == map_frame_id_)
  {
   
    idx_ = 0;
    std::cout<<"origin path size: "<<new_path.poses.size()<<std::endl;
    if (new_path.poses.size() > 0)
    { 
      //path dense
      double pathdensity = 0.1;
      double dis = 0, ang = 0;
      double margin = pathdensity * 0.01;
      double remaining = 0;
      int nPoints = 0;
      path_ = new_path;
      path_.poses.clear();
      path_.poses.push_back(new_path.poses[0]);
      size_t start = 0, next = 1;
      while (next < new_path.poses.size())
      {
          dis += hypot(new_path.poses[next].pose.position.x - new_path.poses[next-1].pose.position.x, new_path.poses[next].pose.position.y - new_path.poses[next-1].pose.position.y) + remaining;
          ang = atan2(new_path.poses[next].pose.position.y - new_path.poses[start].pose.position.y, new_path.poses[next].pose.position.x - new_path.poses[start].pose.position.x);

          if (dis < pathdensity - margin)
          {
              next++;
              remaining = 0;
          } else if (dis > (pathdensity + margin))
          {
              geometry_msgs::PoseStamped point_start = new_path.poses[start];
              nPoints = dis / pathdensity;
              for (int j = 0; j < nPoints; j++)
              {
                  point_start.pose.position.x = point_start.pose.position.x + pathdensity * cos(ang);
                  point_start.pose.position.y = point_start.pose.position.y + pathdensity * sin(ang);
                  point_start.pose.orientation = tf::createQuaternionMsgFromYaw(ang);
                  path_.poses.push_back(point_start);
              }
              remaining = dis - nPoints * pathdensity;
              start++;
              new_path.poses[start].pose.position = point_start.pose.position;
              dis = 0;
              next++;
          } else {
              dis = 0;
              remaining = 0;
              path_.poses.push_back(new_path.poses[next]);
              next++;
              start = next - 1;
          }
      }

      //pathyaw caculate
      if (path_.poses.size() >= 2)
      {
        if (path_.poses.size() == 2) {
          double yaw = cast_from_0_to_2PI_Angle(atan2(path_.poses[1].pose.position.y - path_.poses[0].pose.position.y, path_.poses[1].pose.position.x - path_.poses[0].pose.position.x));//求两点间的角度，将当前角度转换到0～2pi
          path_.poses[0].pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
          path_.poses[1].pose.orientation = path_.poses[0].pose.orientation;
        }

        double yaw = cast_from_0_to_2PI_Angle(atan2(path_.poses[1].pose.position.y - path_.poses[0].pose.position.y, path_.poses[1].pose.position.x - path_.poses[0].pose.position.x));//求两点间的角度，将当前角度转换到0～2pi
        path_.poses[0].pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

        for (int j = 1; j < path_.poses.size() - 1; j++) {
            double yaw = cast_from_0_to_2PI_Angle(atan2(path_.poses[j + 1].pose.position.y - path_.poses[j].pose.position.y, path_.poses[j + 1].pose.position.x - path_.poses[j].pose.position.x));//求两点间的角度，将当前角度转换到0～2pi
            path_.poses[j].pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        }

        int j = (int)path_.poses.size() - 1;
        path_.poses[j].pose.orientation = path_.poses[j-1].pose.orientation;

      }
      
      goal_reached_ = true;
    }
    else
    {
      // goal_reached_ = false;
      ROS_WARN_STREAM("Received empty path!");
    }
  }
  else
  {
    ROS_WARN_STREAM("The path must be published in the " << map_frame_id_
                    << " frame! Ignoring path in " << new_path.header.frame_id
                    << " frame!");
  }
  
}

KDL::Frame PurePursuit::transformToBaseLink(const geometry_msgs::Pose& pose,
                                            const geometry_msgs::Transform& tf)
{
  // Pose in global (map) frame
  KDL::Frame F_map_pose(KDL::Rotation::Quaternion(pose.orientation.x,
                                                  pose.orientation.y,
                                                  pose.orientation.z,
                                                  pose.orientation.w),
                        KDL::Vector(pose.position.x,
                                    pose.position.y,
                                    pose.position.z));

  // Robot (base_link) in global (map) frame
  KDL::Frame F_map_tf(KDL::Rotation::Quaternion(tf.rotation.x,
                                                tf.rotation.y,
                                                tf.rotation.z,
                                                tf.rotation.w),
                      KDL::Vector(tf.translation.x,
                                  tf.translation.y,
                                  tf.translation.z));

  // TODO: See how the above conversions can be done more elegantly
  // using tf2_kdl and tf2_geometry_msgs

  return F_map_tf.Inverse()*F_map_pose;
}

void PurePursuit::run()
{
  ros::spin();
}

void PurePursuit::reconfigure(pure_pursuit::PurePursuitConfig &config, uint32_t level)
{
  // v_max_ = config.max_linear_velocity;
}

int main(int argc, char**argv)
{
  ros::init(argc, argv, "pure_p");

  PurePursuit controller;
  controller.run();

  return 0;
}