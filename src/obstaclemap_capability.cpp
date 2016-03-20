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


#include <hector_move_group_plugins/obstaclemap_capability.h>
#include <moveit/move_group/capability_names.h>

#include <octomap/octomap.h>
#include <octomap_ros/conversions.h>

move_group::ObstaclemapCapability::ObstaclemapCapability():
  MoveGroupCapability("ObstaclemapCapability")
{
}

void move_group::ObstaclemapCapability::initialize()
{

  node_handle_.param("octomap_min_distance_to_obstacle", octo_min_distance_ ,0.05);

  //ros::AdvertiseServiceOptions ops=ros::AdvertiseServiceOptions::create<hector_nav_msgs::GetDistanceToObstacle>("get_distance_to_obstacle", boost::bind(&move_group::ObstaclemapCapability::lookupServiceCallback, this,_1,_2),ros::VoidConstPtr(),&service_queue_);
  //dist_lookup_srv_server_ = node_handle_.advertiseService(ops);


  m_markerPub = node_handle_.advertise<visualization_msgs::MarkerArray>("distance_to_obstacle_debug_markers", 1, false);

  octomap_cv.reset(new octomap_cv_interface::OctomapCvInterface());


  tf_ = this->context_->planning_scene_monitor_->getTFClient();

  debug_img_provider_.reset(new CvDebugProvider(node_handle_));

  service_thread_=boost::thread(boost::bind(&move_group::ObstaclemapCapability::serviceThread,this));

}

void move_group::ObstaclemapCapability::serviceThread(){
  ros::Rate rate(1);
  while (ros::ok()){
    //service_queue_.callAvailable(ros::WallDuration(1.0));

    cv::Mat cv_img;
    cv::Mat free_map;

    {
      planning_scene_monitor::LockedPlanningSceneRO ls (context_->planning_scene_monitor_);

      // Below is inspired by
      // https://github.com/ros-planning/moveit_core/blob/jade-devel/planning_scene/src/planning_scene.cpp#L850
      // and quite hacky. There should be a better way to access the octomap (at least read-only)?
      collision_detection::CollisionWorld::ObjectConstPtr map = ls.getPlanningSceneMonitor()->getPlanningScene()->getWorld()->getObject("<octomap>");
      const shapes::OcTree* octree_shape = static_cast<const shapes::OcTree*>(map->shapes_[0].get());
      const boost::shared_ptr<const octomap::OcTree> octree_ = octree_shape->octree;

      octomap_cv->setOctree(octree_.get());

      octomap_cv->retrieveHeightMap(cv_img, free_map);
    }

    cv::Mat inpaint_lib;
    cv_image_convert::getInpaintedImage(cv_img, inpaint_lib, -0.5, 0.5);





    rate.sleep();
  }
}



#include <class_loader/class_loader.h>
CLASS_LOADER_REGISTER_CLASS(move_group::ObstaclemapCapability, move_group::MoveGroupCapability)
