/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Poznan University of Technology
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
 *     * Neither the name of the Poznan University of Technology nor the names
 *       of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
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

/*
 * Author: Mateusz Przybyla
 */

#pragma once

#include <boost/circular_buffer.hpp>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_listener.h>

namespace obstacle_detector
{

class ScanRectifier
{
public:
  ScanRectifier(ros::NodeHandle& nh, ros::NodeHandle& nh_local);

private:
  bool updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr scan);
  void odomCallback(const nav_msgs::Odometry::ConstPtr odom_msg);
    
  void initialize() { std_srvs::Empty empt; updateParams(empt.request, empt.response); }

  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;

  ros::Subscriber scan_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher scan_pub_;
  ros::ServiceServer params_srv_;

  tf::TransformListener tf_;

  boost::circular_buffer<nav_msgs::Odometry> odoms_;
  sensor_msgs::LaserScan scan_prototype_;
  geometry_msgs::Pose2D scanner_in_base_tf_;

  int num_ranges_;
  std::vector<float> ranges_;
  std::vector<geometry_msgs::Point> points_;

  // Parameters
  bool p_active_;

  double p_scan_rate_;
  double p_odom_rate_;

  int p_odom2scan_ratio_;

  std::string p_robot_frame_;
  std::string p_scanner_frame_;
};

} // namespace obstacle_detector
