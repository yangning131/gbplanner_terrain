#include "bag_reader.hpp"
#include "txt_reader.hpp"
#include <decomp_ros_utils/data_ros_utils.h>
#include <ros/ros.h>
#include <decomp_util/seed_decomp.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/point_cloud_conversion.h>

std_msgs::Header header_;

    sensor_msgs::PointCloud2 cloud3;
    bool receive = false;

    void cloudHandler(const sensor_msgs::PointCloud2::ConstPtr& laserCloudMsg)
    {
        // if (pubLaserCloud.getNumSubscribers() == 0)
        //     return;

        // try{
        //     listener.waitForTransform("map", "velodyne", laserCloudMsg->header.stamp, ros::Duration(1.0));
        //     listener.lookupTransform("map", "velodyne", laserCloudMsg->header.stamp, transform);
        // } 
        // catch (tf::TransformException ex){
        //     return;
        // }
        
        // pcl::PointCloud<pcl::PointXYZL> pc;
        // pcl::PointCloud<pcl::PointXYZL> pc_global;
        // pcl::fromROSMsg(*cloud, pc);

        // cloudHeader = laserCloudMsg->header;

         //pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);

        // newCloudFlag = true;
        //std::cout<<"recevie"<<std::endl;
        receive = true;
        cloud3 = *laserCloudMsg;
    }


int main(int argc, char ** argv){
  ros::init(argc, argv, "test");
  ros::NodeHandle nh("~");
  
  ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud>("cloud", 1, true);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("markers", 1, true);
  ros::Publisher seed_pub = nh.advertise<sensor_msgs::PointCloud>("seed", 1, true);
  ros::Publisher es_pub = nh.advertise<decomp_ros_msgs::EllipsoidArray>("ellipsoid_array", 1, true);
  ros::Publisher poly_pub = nh.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedron_array", 1, true);
  
  ros::Subscriber cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/planning/registered_cloud", 1000,cloudHandler);
  
  header_.frame_id = std::string("map");
  std::string file_name, topic_name, marker_name, seed_file;
  
  nh.param("seed_file", seed_file, std::string("seed.txt"));
  nh.param("bag_file", file_name, std::string("voxel_map"));
  nh.param("bag_topic", topic_name, std::string("voxel_map"));
  nh.param("bag_marker", marker_name, std::string("voxel_map"));
  //Read the point cloud from bag
  
  //sensor_msgs::PointCloud2 cloud2 = read_bag<sensor_msgs::PointCloud2>(file_name, topic_name);
  //Convert into vector of Eigen
 ros::Rate loop_rate(0.8);
  while(1)
  {

  if(receive)
  {

  sensor_msgs::PointCloud cloud;
  sensor_msgs::convertPointCloud2ToPointCloud(cloud3, cloud);



  // std::cout<<"recevie"<<std::endl;

  cloud.header = header_;
  map_pub.publish(cloud);

  vec_Vec2f obs = DecompROS::cloud_to_2vec(cloud);
  



  visualization_msgs::MarkerArray markers = read_bag<visualization_msgs::MarkerArray>(file_name, marker_name);
  for(auto & it: markers.markers)
    it.header = header_;
  marker_pub.publish(markers);

  //Read path from txt
  vec_Vec2f seeds;
  if(!read_path<2>(seed_file, seeds))
    ROS_ERROR("Fail to read seeds!");

  sensor_msgs::PointCloud seed_msg = DecompROS::vec_to_cloud(seeds);
  seed_msg.header = header_;
  seed_pub.publish(seed_msg);

  vec_E<Ellipsoid2D> es;
  vec_E<Polyhedron2D> polys;

  
  for(const auto& it: seeds) {
    //Using seed decomposition
    SeedDecomp2D decomp_util(it);
   
    vec_Vec2f obs_back(obs);
    
    // for(int i = 0;i<360;i++)
    // { 
    //   Vec2f medie;
    //   medie(0) = it(0)+ 8.0*std::cos(i*3.14/180);
    //   medie(1) = it(1) + 8.0*std::sin(i*3.14/180);
    //   obs_back.emplace_back(medie);
    // }

    // for(int i = 0;i<360;i++)
    // { 
    //   Vec2f medie;
    //   medie(0) = 18+ 2.5*std::cos(i*3.14/180);
    //   medie(1) = 0.9 + 2.5*std::sin(i*3.14/180);
    //   obs_back.emplace_back(medie);
    // }
    


    decomp_util.set_obs(obs_back);
    decomp_util.dilate(1.0);

    es.push_back(decomp_util.get_ellipsoid());
    polys.push_back(decomp_util.get_polyhedron());


    /*new*/

      vec_Vec2f  path1;
      path1.emplace_back(Vec2f(5,2.5));
      path1.emplace_back(Vec2f(5,-2.5));
      auto poly = decomp_util.get_polyhedron();
      LinearConstraint2D cs(it, poly.hyperplanes());

    if(cs.inside(path1[0]))
      std::cout << " is inside!" << std::endl;
    else
      std::cout << " is outside!" << std::endl;

               if(cs.inside(path1[1]))
      std::cout << "1 is inside!" << std::endl;
    else
      std::cout << "1 is outside!" << std::endl;


    /*new*/



  }

  //Publish visualization msgs
  decomp_ros_msgs::EllipsoidArray es_msg = DecompROS::ellipsoid_array_to_ros(es);
  es_msg.header = header_;
  es_pub.publish(es_msg);

  decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(polys);
  poly_msg.header = header_;
 

  // for(auto po:poly_msg)
  //   {
  //     int a = po.size();
  //     std::cout<<"a: "<<a<<std::endl;
  //   }

   poly_pub.publish(poly_msg);
  }
  receive = false;

  /*
  vec_LinearConstraint3f cs = decomp_util.get_constraints();
  for(int i = 0; i < cs.size(); i++) {
    MatD3f A = cs[i].first;
    VecDf b = cs[i].second;

    printf("i: %d\n", i);
    std::cout << "start: " << (A*path[i]-b).transpose() << std::endl;
    std::cout << "end: " << (A*path[i+1]-b).transpose() << std::endl;
  }
  */
  ros::spinOnce();
  loop_rate.sleep();
  }
   std::cout<<"recevie000"<<std::endl;
  
  ros::spin();

  return 0;
}
