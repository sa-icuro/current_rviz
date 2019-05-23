/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>

#include <rviz/display_context.h>
#include <rviz/properties/string_property.h>

#include <OgreCamera.h>
#include <OgrePlane.h>
#include <OgreRay.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>


#include <rviz/render_panel.h>
#include <rviz/selection/selection_handler.h>
#include <rviz/selection/selection_manager.h>
#include <rviz/view_controller.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/load_resource.h>
#include <rviz/properties/bool_property.h>

#include "set_safepoint_tool.h"

int counting = 0;

namespace ubiquity_tools
{
  
  SetSafepointTool::SetSafepointTool()
  {
    shortcut_key_ = 'w';

    topic_property_ = new rviz::StringProperty( "Topic", "goal",
                                          "The topic on which to publish Safepoints.",
                                          getPropertyContainer(), SLOT( updateTopic() ), this );
  }

  void SetSafepointTool::onInitialize()
  {
    PoseTool::onInitialize();
    setName( "Set Safepoints" );
    updateTopic();
  }

  void SetSafepointTool::updateTopic()
  {
    pub_ = nh_.advertise<geometry_msgs::PoseStamped>("safepoint_data", 1 );
  }

  void SetSafepointTool::onPoseSet(double x, double y, double theta)
  {
    std::string fixed_frame = context_->getFixedFrame().toStdString();
    tf::Quaternion quat;
    quat.setRPY(0.0, 0.0, theta);
    tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(quat, tf::Point(x, y, 0.0)), ros::Time::now(), fixed_frame);
    geometry_msgs::PoseStamped goal;
    tf::poseStampedTFToMsg(p, goal);
    
    pub_.publish(goal);
  }

} // end namespace ubiquity_tools

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( ubiquity_tools::SetSafepointTool, rviz::Tool )
