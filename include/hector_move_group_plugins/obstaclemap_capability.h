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

#ifndef MOVEIT_MOVE_GROUP_OBSTACLEMAP_CAPABILITY_
#define MOVEIT_MOVE_GROUP_OBSTACLEMAP_CAPABILITY_

#include <moveit/move_group/move_group_capability.h>

#include <octomap_proc_common/octomap_cv_interface.h>

#include <cv_debug_provider/cv_debug_provider.h>

namespace move_group
{

class ObstaclemapCapability : public MoveGroupCapability
{
public:

  ObstaclemapCapability();

  virtual void initialize();

private:
  void serviceThread();

  ros::Publisher m_markerPub;

  ros::CallbackQueue service_queue_;
  boost::thread service_thread_;

  boost::shared_ptr<tf::Transformer> tf_;
  boost::shared_ptr<octomap_cv_interface::OctomapCvInterface> octomap_cv;

  double octo_min_distance_;
  std::string target_frame_;

  boost::shared_ptr<CvDebugProvider> debug_img_provider_;

};

}

#endif
