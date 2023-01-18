#ifndef DECOMP_ROS_UTILS_H
#define DECOMP_ROS_UTILS_H

#include <decomp_geometry/ellipsoid.h>
#include <decomp_geometry/polyhedron.h>
#include <sensor_msgs/PointCloud.h>
#include <decomp_ros_msgs/PolyhedronArray.h>
#include <decomp_ros_msgs/EllipsoidArray.h>
#include <nav_msgs/Path.h>

namespace DecompROS {

template <int Dim> nav_msgs::Path vec_to_path(const vec_Vecf<Dim> &vs) {
  nav_msgs::Path path;
  for (const auto& it : vs) {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = it(0);
    pose.pose.position.y = it(1);
    pose.pose.position.z = Dim == 2 ? 0 : it(2);
    pose.pose.orientation.w = 1.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;

    path.poses.push_back(pose);
  }

  return path;
}

inline sensor_msgs::PointCloud vec_to_cloud(const vec_Vec3f &pts) {
  sensor_msgs::PointCloud cloud;
  cloud.points.resize(pts.size());

  for (unsigned int i = 0; i < pts.size(); i++) {
    cloud.points[i].x = pts[i](0);
    cloud.points[i].y = pts[i](1);
    cloud.points[i].z = pts[i](2);
  }
  return cloud;
}

inline sensor_msgs::PointCloud vec_to_cloud(const vec_Vec2f &pts) {
  sensor_msgs::PointCloud cloud;
  cloud.points.resize(pts.size());

  for (unsigned int i = 0; i < pts.size(); i++) {
    cloud.points[i].x = pts[i](0);
    cloud.points[i].y = pts[i](1);
     cloud.points[i].z = 0.5;
  }

  return cloud;
}

// inline sensor_msgs::PointCloud vec_to_cloud(const vec_Vec2f &pts) { //chage
//   sensor_msgs::PointCloud cloud;
//   cloud.points.resize(pts.size()+7);

//   for (unsigned int i = 0; i < pts.size(); i++) {
//     cloud.points[i].x = pts[i](0);
//     cloud.points[i].y = pts[i](1);
//      cloud.points[i].z = 0.5;
//   }
//     cloud.points[pts.size()].x = 16;
//     cloud.points[pts.size()].y = 0;
//      cloud.points[pts.size()].z = 0.5;
     
//          cloud.points[pts.size()+1].x = 20;
//     cloud.points[pts.size()+1].y = 0.9;
//      cloud.points[pts.size()+1].z = 0.5;

//          cloud.points[pts.size()+2].x = 17;
//     cloud.points[pts.size()+2].y = -0.9;
//      cloud.points[pts.size()+2].z = 0.5;

//          cloud.points[pts.size()+3].x = 17.5;
//     cloud.points[pts.size()+3].y = -0.5;
//      cloud.points[pts.size()+3].z = 0.5;

//          cloud.points[pts.size()+4].x = 18;
//     cloud.points[pts.size()+4].y = 2.5;
//      cloud.points[pts.size()+4].z = 0.5;

//          cloud.points[pts.size()+5].x = 17.6;
//     cloud.points[pts.size()+5].y = 2.4;
//      cloud.points[pts.size()+5].z = 0.5;

//          cloud.points[pts.size()+6].x = 17;
//     cloud.points[pts.size()+6].y = 1;
//      cloud.points[pts.size()+6].z = 0.5;


     
  
//   return cloud;
// }

inline vec_Vec3f cloud_to_vec(const sensor_msgs::PointCloud &cloud) {
  vec_Vec3f pts;
  pts.resize(cloud.points.size());
  for (unsigned int i = 0; i < cloud.points.size(); i++) {
    pts[i](0) = cloud.points[i].x;
    pts[i](1) = cloud.points[i].y;
    pts[i](2) = cloud.points[i].z;
  }

  return pts;
}

inline vec_Vec2f cloud_to_2vec(const sensor_msgs::PointCloud &cloud) {
  vec_Vec2f pts;
  pts.resize(cloud.points.size());
  for (unsigned int i = 0; i < cloud.points.size(); i++) {
    if(cloud.points[i].z>0.05&&cloud.points[i].z<0.8)
    {
    //  if(cloud.points[i].y<-10) continue;
    pts[i](0) = cloud.points[i].x;
    pts[i](1) = cloud.points[i].y;
    // pts[i](2) = cloud.points[i].z;
    }

  }

  return pts;
}

inline Polyhedron3D ros_to_polyhedron(const decomp_ros_msgs::Polyhedron& msg){
  Polyhedron3D poly;
  for(unsigned int i = 0; i < msg.points.size(); i++){
    Vec3f pt(msg.points[i].x,
             msg.points[i].y,
             msg.points[i].z);
    Vec3f n(msg.normals[i].x,
            msg.normals[i].y,
            msg.normals[i].z);
    poly.add(Hyperplane3D(pt, n));
  }
  return poly;
}

inline vec_E<Polyhedron3D> ros_to_polyhedron_array(const decomp_ros_msgs::PolyhedronArray& msg) {
  vec_E<Polyhedron3D> polys(msg.polyhedrons.size());

  for(size_t i = 0; i < msg.polyhedrons.size(); i++)
    polys[i] = ros_to_polyhedron(msg.polyhedrons[i]);

  return polys;
}

inline decomp_ros_msgs::Polyhedron polyhedron_to_ros(const Polyhedron2D& poly){
  decomp_ros_msgs::Polyhedron msg;
  for (const auto &p : poly.hyperplanes()) {
    geometry_msgs::Point pt, n;
    pt.x = p.p_(0);
    pt.y = p.p_(1);
    pt.z = 0;
    n.x = p.n_(0);
    n.y = p.n_(1);
    n.z = 0;
    msg.points.push_back(pt);
    msg.normals.push_back(n);
  }

  geometry_msgs::Point pt1, n1;
  pt1.x = -100, pt1.y = -100, pt1.z = 0.01;
  n1.x = 0, n1.y = 0, n1.z = 1;
  msg.points.push_back(pt1);
  msg.normals.push_back(n1);

  geometry_msgs::Point pt2, n2;
  pt2.x = -100, pt2.y = -100, pt2.z = -0.01;
  n2.x = 0, n2.y = 0, n2.z = -1;
  msg.points.push_back(pt2);
  msg.normals.push_back(n2);

  return msg;
}

inline decomp_ros_msgs::Polyhedron polyhedron_to_ros(const Polyhedron3D& poly){
  decomp_ros_msgs::Polyhedron msg;
  for (const auto &p : poly.hyperplanes()) {
    geometry_msgs::Point pt, n;
    pt.x = p.p_(0);
    pt.y = p.p_(1);
    pt.z = p.p_(2);
    n.x = p.n_(0);
    n.y = p.n_(1);
    n.z = p.n_(2);
    msg.points.push_back(pt);
    msg.normals.push_back(n);
  }

  return msg;
}


template <int Dim>
decomp_ros_msgs::PolyhedronArray polyhedron_array_to_ros(const vec_E<Polyhedron<Dim>>& vs){
  decomp_ros_msgs::PolyhedronArray msg;
  for (const auto &v : vs)
    msg.polyhedrons.push_back(polyhedron_to_ros(v));
  return msg;
}

template <int Dim>
decomp_ros_msgs::EllipsoidArray ellipsoid_array_to_ros(const vec_E<Ellipsoid<Dim>>& Es) {
  decomp_ros_msgs::EllipsoidArray ellipsoids;
  for (unsigned int i = 0; i < Es.size(); i++) {
    decomp_ros_msgs::Ellipsoid ellipsoid;
    auto d = Es[i].d();
    ellipsoid.d[0] = d(0);
    ellipsoid.d[1] = d(1);
    ellipsoid.d[2] = Dim == 2 ? 0:d(2);

    auto C = Es[i].C();
    for (int x = 0; x < 3; x++) {
      for (int y = 0; y < 3; y++) {
        if(x < Dim && y < Dim)
          ellipsoid.E[3 * x + y] = C(x, y);
        else
          ellipsoid.E[3 * x + y] = 0;
      }
    }
    ellipsoids.ellipsoids.push_back(ellipsoid);
  }

  return ellipsoids;
}

}

#endif
