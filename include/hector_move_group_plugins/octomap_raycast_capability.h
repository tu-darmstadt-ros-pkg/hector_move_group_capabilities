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

#ifndef MOVEIT_MOVE_GROUP_OCTOMAP_RAYCAST_CAPABILITY_
#define MOVEIT_MOVE_GROUP_OCTOMAP_RAYCAST_CAPABILITY_

#include <moveit/move_group/move_group_capability.h>

#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <hector_nav_msgs/GetDistanceToObstacle.h>

#include <dynamic_reconfigure/server.h>
#include <hector_move_group_capabilities/OctomapRaycastCapabilityConfig.h>

namespace move_group
{

class OctomapRaycastCapability : public MoveGroupCapability
{
public:

  OctomapRaycastCapability();

  virtual void initialize();

  void dynRecParamCallback(hector_move_group_capabilities::OctomapRaycastCapabilityConfig &config, uint32_t level);

private:
  void serviceThread();
  bool lookupServiceCallback(hector_nav_msgs::GetDistanceToObstacle::Request  &req,
                             hector_nav_msgs::GetDistanceToObstacle::Response &res );

  void cast_ray_mod_direction(
                                              const octomap::point3d& origin,
                                              const octomap::OcTree& octree,
                                              const tf::Vector3& direction,
                                              double pitch,
                                              double yaw,
                                              octomap::point3d& end_point);

  void get_endpoints(const octomap::point3d& origin,
                     const octomap::OcTree& octree,
                     const float reference_distance,
                     const tf::Vector3& direction,
                     std::vector<octomap::point3d>& directions,
                     std::vector<octomap::point3d>& endPoints,
                     int n);

  //void visTimerCallback(const ros::TimerEvent& event);

  //ros::Timer vis_timer_;
  //ros::Publisher octomap_full_pub_;
  boost::shared_ptr< dynamic_reconfigure::Server<hector_move_group_capabilities::OctomapRaycastCapabilityConfig> >dyn_rec_server_;


  ros::ServiceServer dist_lookup_srv_server_;

  ros::Publisher m_markerPub;

  ros::CallbackQueue service_queue_;
  boost::thread service_thread_;

  std::shared_ptr<tf2_ros::Buffer> tf_;

  double octo_min_distance_;
  double octo_max_distance_;
  double secondary_rays_max_dist_;
  double secondary_rays_opening_angle_;

  std::string target_frame_;

//  bool clearOctomap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

//  ros::ServiceServer service_;
};

}

#endif
