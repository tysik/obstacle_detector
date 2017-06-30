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

#include "obstacle_detector/scans_merger.h"
#include "obstacle_detector/utilities/math_utilities.h"

using namespace obstacle_detector;
using namespace std;

ScansMerger::ScansMerger(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : nh_(nh), nh_local_(nh_local) {
  p_active_ = false;

  front_scan_received_ = false;
  rear_scan_received_ = false;

  front_scan_error_ = false;
  rear_scan_error_ = false;

  params_srv_ = nh_local_.advertiseService("params", &ScansMerger::updateParams, this);

  ROS_INFO_STREAM("[Scans Merger]: Waiting for first scans.");
  boost::shared_ptr<sensor_msgs::LaserScan const> scan;

  scan = ros::topic::waitForMessage<sensor_msgs::LaserScan>("front_scan", nh_);
  front_scan_frame_id_ = scan->header.frame_id;

  scan = ros::topic::waitForMessage<sensor_msgs::LaserScan>("rear_scan", nh_);
  rear_scan_frame_id_ = scan->header.frame_id;
  ROS_INFO_STREAM("[Scans Merger]: Acquired first scans.");

  initialize();
}

bool ScansMerger::updateParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  bool prev_active = p_active_;

  nh_local_.param<bool>("active", p_active_, true);
  nh_local_.param<bool>("publish_scan", p_publish_scan_, true);
  nh_local_.param<bool>("publish_pcl", p_publish_pcl_, false);

  nh_local_.param<int>("ranges_num", p_ranges_num_, 1000);

  nh_local_.param<double>("min_scanner_range", p_min_scanner_range_, 0.05);
  nh_local_.param<double>("max_scanner_range", p_max_scanner_range_, 10.0);
  nh_local_.param<double>("max_x_range", p_max_x_range_,  10.0);
  nh_local_.param<double>("min_x_range", p_min_x_range_, -10.0);
  nh_local_.param<double>("max_y_range", p_max_y_range_,  10.0);
  nh_local_.param<double>("min_y_range", p_min_y_range_, -10.0);

  nh_local_.param<std::string>("frame_id", p_frame_id_, "scanner_base");

  try {
    ROS_INFO_STREAM("[Scans Merger]: Waiting for transformations.");
    tf::TransformListener tf;
    tf::StampedTransform transform;

    tf.waitForTransform(p_frame_id_, front_scan_frame_id_, ros::Time(0), ros::Duration(10.0));
    tf.lookupTransform(p_frame_id_, front_scan_frame_id_, ros::Time(0), transform);

    front_tf_.x = transform.getOrigin().getX();
    front_tf_.y = transform.getOrigin().getY();
    front_tf_.theta = tf::getYaw(transform.getRotation());

    tf.waitForTransform(p_frame_id_, rear_scan_frame_id_, ros::Time(0), ros::Duration(10.0));
    tf.lookupTransform(p_frame_id_, rear_scan_frame_id_, ros::Time(0), transform);

    rear_tf_.x = transform.getOrigin().getX();
    rear_tf_.y = transform.getOrigin().getY();
    rear_tf_.theta = tf::getYaw(transform.getRotation());
    ROS_INFO_STREAM("[Scans Merger]: Acquired transformations.");
  }
  catch (tf::TransformException ex) {
    throw ex.what();
  }

  if (p_active_ != prev_active) {
    if (p_active_) {
      front_scan_sub_ = nh_.subscribe("front_scan", 10, &ScansMerger::frontScanCallback, this);
      rear_scan_sub_ = nh_.subscribe("rear_scan", 10, &ScansMerger::rearScanCallback, this);
      scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 10);
      pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud>("pcl", 10);
    }
    else {
      front_scan_sub_.shutdown();
      rear_scan_sub_.shutdown();
      scan_pub_.shutdown();
      pcl_pub_.shutdown();
    }
  }

  return true;
}

void ScansMerger::frontScanCallback(const sensor_msgs::LaserScan::ConstPtr front_scan) {
  if (front_scan_received_)
    points_.clear();

  if (!rear_scan_received_ || rear_scan_error_)
    stamp_ = front_scan->header.stamp;

  geometry_msgs::Point32 local_point, base_point;
  float phi = front_scan->angle_min;

  for (const float r : front_scan->ranges) {
    if (r > front_scan->range_min && r < front_scan->range_max) {
      local_point.x = r * cos(phi);
      local_point.y = r * sin(phi);

      base_point = transformPoint(local_point, front_tf_.x, front_tf_.y, front_tf_.theta);

      if (checkPointInLimits(base_point, p_min_x_range_, p_max_x_range_, p_min_y_range_, p_max_y_range_))
        points_.push_back(base_point);
    }

    phi += front_scan->angle_increment;
  }

  front_scan_received_ = true;
  front_scan_error_ = false;

  if (rear_scan_received_ || rear_scan_error_)
    publishAll();
  else
    rear_scan_error_ = true;
}

void ScansMerger::rearScanCallback(const sensor_msgs::LaserScan::ConstPtr rear_scan) {
  if (rear_scan_received_)
    points_.clear();

  if (!front_scan_received_ || front_scan_error_)
    stamp_ = rear_scan->header.stamp;

  geometry_msgs::Point32 local_point, base_point;
  float phi = rear_scan->angle_min;

  for (const float r : rear_scan->ranges) {
    if (r > rear_scan->range_min && r < rear_scan->range_max) {
      local_point.x = r * cos(phi);
      local_point.y = r * sin(phi);

      base_point = transformPoint(local_point, rear_tf_.x, rear_tf_.y, rear_tf_.theta);

      if (checkPointInLimits(base_point, p_min_x_range_, p_max_x_range_, p_min_y_range_, p_max_y_range_))
        points_.push_back(base_point);
    }

    phi += rear_scan->angle_increment;
  }

  rear_scan_received_ = true;
  rear_scan_error_ = false;

  if (front_scan_received_ || front_scan_error_)
    publishAll();
  else
    front_scan_error_ = true;
}

void ScansMerger::publishScan() {
  sensor_msgs::LaserScanPtr scan_msg(new sensor_msgs::LaserScan);

  scan_msg->header.frame_id = p_frame_id_;
  scan_msg->header.stamp = stamp_;
  scan_msg->angle_min = -M_PI;
  scan_msg->angle_max = M_PI;
  scan_msg->angle_increment = 2.0 * M_PI / (p_ranges_num_ - 1);
  scan_msg->time_increment = 0.0;
  scan_msg->scan_time = 0.1;
  scan_msg->range_min = p_min_scanner_range_;
  scan_msg->range_max = p_max_scanner_range_;

  ranges_.assign(p_ranges_num_, 2.0 * p_max_scanner_range_);

  for (auto& point : points_) {
    float angle = atan2(point.y, point.x);
    float range = sqrt(pow(point.x, 2.0) + pow(point.y, 2.0));

    int idx = static_cast<int>(p_ranges_num_ * (angle + M_PI) / (2.0 * M_PI));

    if (ranges_[idx] > range)
      ranges_[idx] = range;
  }

  for (int jdx = 0; jdx < ranges_.size(); ++jdx)
    if (ranges_[jdx] < p_min_scanner_range_ || ranges_[jdx] > p_max_scanner_range_)
      ranges_[jdx] = nan("");

  scan_msg->ranges = ranges_;

  scan_pub_.publish(scan_msg);
}

void ScansMerger::publishPCL() {
  sensor_msgs::PointCloudPtr pcl_msg(new sensor_msgs::PointCloud);

  pcl_msg->header.frame_id = p_frame_id_;
  pcl_msg->header.stamp = ros::Time::now();
  pcl_msg->points = points_;

  pcl_pub_.publish(pcl_msg);
}

void ScansMerger::publishAll() {
  if (p_publish_scan_)
    publishScan();

  if (p_publish_pcl_)
    publishPCL();

  front_scan_received_ = false;
  rear_scan_received_ = false;

  points_.clear();
}
