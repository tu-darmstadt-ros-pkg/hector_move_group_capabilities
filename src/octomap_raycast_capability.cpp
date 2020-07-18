/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Stefan Kohlbrecher, TU Darmstadt
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of TU Darmstadt, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#include <hector_move_group_plugins/octomap_raycast_capability.h>
#include <moveit/move_group/capability_names.h>

#include <octomap/octomap.h>
#include <octomap_ros/conversions.h>

#include <angles/angles.h>

move_group::OctomapRaycastCapability::OctomapRaycastCapability():
  MoveGroupCapability("OctomapRaycastCapability")
{
}

void move_group::OctomapRaycastCapability::initialize()
{
  dyn_rec_server_.reset(new dynamic_reconfigure::Server<hector_move_group_capabilities::OctomapRaycastCapabilityConfig>(ros::NodeHandle("~/octomap_raycast_capability")));
  dyn_rec_server_->setCallback(boost::bind(&move_group::OctomapRaycastCapability::dynRecParamCallback, this, _1, _2));

  //node_handle_.param("octomap_min_distance_to_obstacle", octo_min_distance_ ,0.05);
  //node_handle_.param("octomap_max_distance_to_obstacle", octo_max_distance_ ,3.5);

  ros::AdvertiseServiceOptions ops=ros::AdvertiseServiceOptions::create<hector_nav_msgs::GetDistanceToObstacle>("get_distance_to_obstacle", boost::bind(&move_group::OctomapRaycastCapability::lookupServiceCallback, this,_1,_2),ros::VoidConstPtr(),&service_queue_);
  dist_lookup_srv_server_ = node_handle_.advertiseService(ops);

  service_thread_=boost::thread(boost::bind(&move_group::OctomapRaycastCapability::serviceThread,this));

  m_markerPub = node_handle_.advertise<visualization_msgs::MarkerArray>("distance_to_obstacle_debug_markers", 1, false);


  tf_ = this->context_->planning_scene_monitor_->getTFClient();
}

void move_group::OctomapRaycastCapability::serviceThread(){
    ros::Rate rate(100.0);
    while (ros::ok()){
        service_queue_.callAvailable(ros::WallDuration(1.0));
        rate.sleep();
    }
}

void move_group::OctomapRaycastCapability::dynRecParamCallback(hector_move_group_capabilities::OctomapRaycastCapabilityConfig &config, uint32_t level)
{
  octo_min_distance_ = config.min_distance_to_obstacle;
  octo_max_distance_ = config.max_distance_to_obstacle;
  secondary_rays_max_dist_ = config.secondary_rays_max_dist;
  secondary_rays_opening_angle_ = angles::from_degrees(config.secondary_rays_opening_angle_deg);
}

bool move_group::OctomapRaycastCapability::lookupServiceCallback(hector_nav_msgs::GetDistanceToObstacle::Request  &req,
                                                                 hector_nav_msgs::GetDistanceToObstacle::Response &res )
{

  ROS_DEBUG("Octomap distance lookup service called");
  geometry_msgs::TransformStamped camera_transform;

  const std::string target_frame = context_->planning_scene_monitor_->getPlanningScene()->getPlanningFrame();

  try{
    camera_transform = tf_->lookupTransform(target_frame, req.point.header.frame_id, req.point.header.stamp, ros::Duration(1.0));
  }catch(tf2::TransformException& e){
    ROS_ERROR("Transform failed in lookup distance service call: %s",e.what());
    return false;
  }

  bool useOutliers = true;

  tf2::Transform camera_transform_tf;
  tf2::fromMsg(camera_transform.transform, camera_transform_tf); 	
  
  tf::Vector3 camera_translation;
  tf::vector3MsgToTF(camera_transform.transform.translation, camera_translation);

  const octomap::point3d origin = octomap::pointTfToOctomap(camera_translation);

  tf2::Vector3 end_point = camera_transform_tf * tf2::Vector3(req.point.point.x, req.point.point.y, req.point.point.z);
  tf2::Vector3 direction_tf2 = end_point - camera_transform_tf.getOrigin();
  tf::Vector3 direction(direction_tf2.x(), direction_tf2.y(), direction_tf2.z());

  const octomap::point3d directionOc = octomap::pointTfToOctomap(direction);
  std::vector<octomap::point3d> endPoints;
  std::vector<float> distances;
  int n=2;
  endPoints.resize(1);
  std::vector<octomap::point3d> directions;
  directions.push_back(directionOc);

  planning_scene_monitor::LockedPlanningSceneRO ls (context_->planning_scene_monitor_);

  // Below is inspired by
  // https://github.com/ros-planning/moveit_core/blob/jade-devel/planning_scene/src/planning_scene.cpp#L850
  // and quite hacky. There should be a better way to access the octomap (at least read-only)?
  collision_detection::CollisionEnv::ObjectConstPtr map = ls.getPlanningSceneMonitor()->getPlanningScene()->getWorld()->getObject("<octomap>");
  const shapes::OcTree* octree_shape = static_cast<const shapes::OcTree*>(map->shapes_[0].get());
  const std::shared_ptr<const octomap::OcTree> octree_ = octree_shape->octree;

  if(octree_->castRay(origin,directions[0],endPoints[0],true,octo_max_distance_)) {
    distances.push_back(origin.distance(endPoints[0]));
  }

  if (distances.size()!=0) {
    int count_outliers;
    endPoints.resize(1+(n*2));
    get_endpoints(origin, *octree_, distances[0], direction, directions, endPoints, n);
    if (useOutliers) {
      double distance_threshold=0.7;
      count_outliers=0;
      for (size_t i =1;i<endPoints.size();i++){
        ROS_DEBUG("distance to checkpoints %f",endPoints[0].distance(endPoints[i]));
        if(endPoints[0].distance(endPoints[i])>distance_threshold) {
          count_outliers++;
        }
      }
      ROS_DEBUG("corner case: number of outliers: %d ",count_outliers);
    }

    res.distance = distances[0];

    res.end_point.header.frame_id = target_frame;
    res.end_point.header.stamp = req.point.header.stamp;
    res.end_point.point.x = endPoints[0].x();
    res.end_point.point.y = endPoints[0].y();
    res.end_point.point.z = endPoints[0].z();

    if (res.distance < octo_min_distance_) {
      res.distance = -1.0;
      ROS_WARN("Octomap GetDistanceToObstacle got distance under min_Distance -> returning -1.0");
    }

    if (useOutliers) {
      if (count_outliers>=n-1) {
        res.distance=-1.0;
        ROS_DEBUG("Octomap GetDistanceToObstacle Corner Case");
      }
    }
  } else {
    res.distance=-1.0;
  }

  ROS_DEBUG("Result: getDistanceObstacle_Octo: distance: %f",res.distance);
  if (m_markerPub.getNumSubscribers() > 0){
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;
    marker.header.stamp = req.point.header.stamp;
    marker.header.frame_id = target_frame;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r= 1.0;
    marker.color.a = 1.0;
    marker.scale.x = 0.02;
    marker.ns ="";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;

    std::vector<geometry_msgs::Point> point_vector;
    for (size_t i = 0; i < endPoints.size(); ++i){
      geometry_msgs::Point tmp = octomap::pointOctomapToMsg(origin);
      point_vector.push_back(tmp);

      tmp = octomap::pointOctomapToMsg(endPoints[i]);
      point_vector.push_back(tmp);
    }
    marker.points=point_vector;
    marker_array.markers.push_back(marker);
    m_markerPub.publish(marker_array);
  }

  return true;
}

void move_group::OctomapRaycastCapability::cast_ray_mod_direction(
                                            const octomap::point3d& origin,
                                            const octomap::OcTree& octree,
                                            const tf::Vector3& direction,
                                            double pitch,
                                            double yaw,
                                            octomap::point3d& end_point)
{
  tf::Vector3 dir_mod = direction;
  dir_mod = dir_mod.rotate(tf::Vector3(0.0 ,0.0, 1.0), yaw);
  dir_mod = dir_mod.rotate(tf::Vector3(0.0, 1.0 ,0.0), pitch);

  const octomap::point3d dir_mod_oc = octomap::pointTfToOctomap(dir_mod);

  octree.castRay(origin, dir_mod_oc, end_point, true, secondary_rays_max_dist_);
}

void move_group::OctomapRaycastCapability::get_endpoints(const octomap::point3d& origin,
                                                         const octomap::OcTree& octree,
                                                         const float reference_distance,
                                                         const tf::Vector3& direction,
                                                         std::vector<octomap::point3d>& directions,
                                                         std::vector<octomap::point3d>& endPoints,
                                                         int n){

  cast_ray_mod_direction(origin, octree, direction,  0.0 , secondary_rays_opening_angle_, endPoints[1]);
  cast_ray_mod_direction(origin, octree, direction,  0.0 ,-secondary_rays_opening_angle_, endPoints[2]);
  cast_ray_mod_direction(origin, octree, direction,  secondary_rays_opening_angle_ , 0.0, endPoints[3]);
  cast_ray_mod_direction(origin, octree, direction, -secondary_rays_opening_angle_ , 0.0, endPoints[4]);
  /*
    tf::Vector3 z_axis(0,0,1);

    double tolerance=octree.getResolution()*0.05;

    for (int i=1;i<=n;i++){
        double angle=std::atan2((octree.getResolution()*i)+tolerance,reference_distance);

        tf::Vector3 direction_z_plus = direction.rotate(z_axis, +angle);
        tf::Vector3 direction_z_minus = direction.rotate(z_axis, -angle);

        const octomap::point3d direction_z_plus_Oc = octomap::pointTfToOctomap(direction_z_plus);
        const octomap::point3d direction_z_minus_Oc = octomap::pointTfToOctomap(direction_z_minus);

        directions.push_back(direction_z_plus_Oc);
        directions.push_back(direction_z_minus_Oc);

        octree.castRay(origin,directions[2*i-1],endPoints[2*i-1],true,5.0);
        octree.castRay(origin,directions[2*i],endPoints[2*i],true,5.0);

    }
    */
}

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(move_group::OctomapRaycastCapability, move_group::MoveGroupCapability)
