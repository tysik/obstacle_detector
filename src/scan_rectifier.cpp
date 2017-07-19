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

#include "obstacle_detector/scan_rectifier.h"

using namespace obstacle_detector;
using namespace std;

ScanRectifier::ScanRectifier(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : nh_(nh), nh_local_(nh_local) {
  p_active_ = false;
  params_srv_ = nh_local_.advertiseService("params", &ScanRectifier::updateParams, this);

//  ROS_INFO_STREAM("[Scan Rectifier]: Waiting for first " << p_odom2scan_ratio_ << " odometry messages.");
//  boost::shared_ptr<nav_msgs::Odometry const> first_odom;
//  for (int i = 0; i < p_odom2scan_ratio_; ++i) {
//    first_odom = ros::topic::waitForMessage<nav_msgs::Odometry>("odom", nh_);
//    odoms_.push_back(*first_odom);
//  }
//  ROS_INFO_STREAM("[Scan Rectifier]: Acquired first " << p_odom2scan_ratio_ << " odometry messages.");

//  ROS_INFO_STREAM("[Scan Rectifier]: Waiting for first odometry message.");
//  boost::shared_ptr<nav_msgs::Odometry const> first_odom;
//  first_odom = ros::topic::waitForMessage<nav_msgs::Odometry>("odom", nh_);
//  odoms_.push_back(*first_odom);
//  ROS_INFO_STREAM("[Scan Rectifier]: Acquired first odometry message.");

//  ROS_INFO_STREAM("[Scan Rectifier]: Waiting for first scan message.");
//  boost::shared_ptr<sensor_msgs::LaserScan const> first_scan;
//  first_scan = ros::topic::waitForMessage<sensor_msgs::LaserScan>("scan", nh_);
//  scan_prototype_ = *first_scan;
//  num_ranges_ = scan_prototype_.ranges.size();
//  scan_prototype_.ranges.clear();
//  ROS_INFO_STREAM("[Scan Rectifier]: Acquired first laser scan.");

  initialize();
}

bool ScanRectifier::updateParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  bool prev_active = p_active_;

  nh_local_.param<bool>("active", p_active_, true);

  nh_local_.param<double>("scan_rate", p_scan_rate_, 10.0);
  nh_local_.param<double>("odom_rate", p_odom_rate_, 100.0);

  p_odom2scan_ratio_ = static_cast<int>(p_odom_rate_ / p_scan_rate_);
  odoms_.set_capacity(p_odom2scan_ratio_);

  nh_local_.param<string>("robot_frame", p_robot_frame_, string("robot"));
  nh_local_.param<string>("scanner_frame", p_scanner_frame_, string("scanner"));

  try {
    ROS_INFO_STREAM("[Scan Rectifier]: Waiting for transformation: " << p_robot_frame_ << " -> " << p_scanner_frame_);
    tf::StampedTransform transform;
    ros::Time now = ros::Time::now();
    tf_.waitForTransform(p_robot_frame_, p_scanner_frame_, now, ros::Duration(10.0));
    tf_.lookupTransform(p_robot_frame_, p_scanner_frame_, now, transform);

    scanner_in_base_tf_.x = transform.getOrigin().getX();
    scanner_in_base_tf_.y = transform.getOrigin().getY();
    scanner_in_base_tf_.theta = tf::getYaw(transform.getRotation());
    ROS_INFO_STREAM("[Scan Rectifier]: Acquired transformation " << p_robot_frame_ << " -> " << p_scanner_frame_);
  }
  catch (tf::TransformException ex) {
    throw ex.what();
  }

  if (p_active_ != prev_active) {
    if (p_active_) {
      scan_sub_ = nh_.subscribe("scan", 10, &ScanRectifier::scanCallback, this);
      odom_sub_ = nh_.subscribe("odom", 10, &ScanRectifier::odomCallback, this);
      scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("rect_scan", 10);
    }
    else {
      scan_sub_.shutdown();
      odom_sub_.shutdown();
      scan_pub_.shutdown();
    }
  }

  return true;
}

void ScanRectifier::scanCallback(const sensor_msgs::LaserScan::ConstPtr scan_msg) {
  if (odoms_.size() < odoms_.capacity())
    return;

  points_.clear();

  double t = 0.0;
  double theta = 0.0;
  double phi = scan_msg->angle_min;

  double dt = scan_msg->time_increment;
  double dphi = scan_msg->angle_increment;
  double dt_offset = scan_msg->header.stamp.toSec() - odoms_[0].header.stamp.toSec();

  int idx_offset = 0;
  if (dt_offset > 0.0)
    idx_offset = static_cast<int>(dt_offset * p_odom_rate_);

  // Temporary velocities in robot coordinate frame
  double u, v, w;
  u = v = w = 0.0;

  // Corrections of point coordinates resulting from motion prediction
  double x_cor, y_cor;
  x_cor = y_cor = 0.0;

  geometry_msgs::Point point;
  for (const float r : scan_msg->ranges) {
    if (r > scan_msg->range_min && r < scan_msg->range_max) {
      int cur_odom_idx = static_cast<int>(p_odom2scan_ratio_ * t / scan_msg->scan_time) + idx_offset;
      if (cur_odom_idx >= odoms_.size())
        cur_odom_idx = odoms_.size() - 1;

      u = odoms_[cur_odom_idx].twist.twist.linear.x;
      v = odoms_[cur_odom_idx].twist.twist.linear.y;
      w = odoms_[cur_odom_idx].twist.twist.angular.z;

      double v_s_x = u - scanner_in_base_tf_.y * w;
      double v_s_y = v + scanner_in_base_tf_.x * w;

      // TODO: Rotate velocities into scanner frame

      if (w != 0.0) {
        x_cor = (v_s_x * sin(theta) + v_s_y * (cos(theta) - 1.0)) / w;
        y_cor = (v_s_x * (1.0 - cos(theta)) + v_s_y * sin(theta)) / w;
      }
      else {
        x_cor = v_s_x * t;
        y_cor = v_s_y * t;
      }

      point.x = r * cos(phi + theta) + x_cor;
      point.y = r * sin(phi + theta) + y_cor;

      points_.push_back(point);
    }

    t += dt;
    phi += dphi;
    theta += w * dt;
  }

  // Prepare and publish scan
  sensor_msgs::LaserScanPtr rect_scan_msg(new sensor_msgs::LaserScan);

  ranges_.assign(scan_msg->ranges.size(), 10.0f * scan_msg->range_max);

  for (auto& point : points_) {
    float angle = atan2(point.y, point.x);
    float range = sqrt(pow(point.x, 2.0) + pow(point.y, 2.0));
\
    int idx = static_cast<int>(scan_msg->ranges.size() * (angle - scan_msg->angle_min) / (scan_msg->angle_max - scan_msg->angle_min));

    if (ranges_[idx] > range)
      ranges_[idx] = range;
  }

  for (int jdx = 0; jdx < scan_msg->ranges.size(); ++jdx)
    if (ranges_[jdx] < scan_msg->range_min || ranges_[jdx] > scan_msg->range_max)
      ranges_[jdx] = nan("");

  rect_scan_msg->header.frame_id = p_scanner_frame_;
  rect_scan_msg->header.stamp = scan_msg->header.stamp;

  rect_scan_msg->angle_min = scan_msg->angle_min;
  rect_scan_msg->angle_max = scan_msg->angle_max;
  rect_scan_msg->angle_increment = scan_msg->angle_increment;

  rect_scan_msg->range_min = scan_msg->range_min;
  rect_scan_msg->range_max = scan_msg->range_max;

  rect_scan_msg->scan_time = scan_msg->scan_time;
  rect_scan_msg->time_increment = scan_msg->time_increment;

  rect_scan_msg-> ranges = ranges_;

  scan_pub_.publish(rect_scan_msg);
}

void ScanRectifier::odomCallback(const nav_msgs::Odometry::ConstPtr odom_msg) {
  odoms_.push_back(*odom_msg);
}
