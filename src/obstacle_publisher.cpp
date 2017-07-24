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

#include "obstacle_detector/obstacle_publisher.h"

using namespace std;
using namespace obstacle_detector;

ObstaclePublisher::ObstaclePublisher(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : nh_(nh), nh_local_(nh_local) {
  p_active_ = false;
  t_ = 0.0;

  timer_ = nh_.createTimer(ros::Duration(1.0), &ObstaclePublisher::timerCallback, this, false, false);
  params_srv_ = nh_local_.advertiseService("params", &ObstaclePublisher::updateParams, this);
  initialize();
}

ObstaclePublisher::~ObstaclePublisher() {
  nh_local_.deleteParam("active");
  nh_local_.deleteParam("reset");

  nh_local_.deleteParam("fusion_example");
  nh_local_.deleteParam("fission_example");

  nh_local_.deleteParam("loop_rate");
  nh_local_.deleteParam("radius_margin");

  nh_local_.deleteParam("x_vector");
  nh_local_.deleteParam("y_vector");
  nh_local_.deleteParam("r_vector");

  nh_local_.deleteParam("vx_vector");
  nh_local_.deleteParam("vy_vector");

  nh_local_.deleteParam("frame_id");
}

bool ObstaclePublisher::updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  bool prev_active = p_active_;

  nh_local_.param<bool>("active", p_active_, true);
  nh_local_.param<bool>("reset", p_reset_, false);

  nh_local_.param<bool>("fusion_example", p_fusion_example_, false);
  nh_local_.param<bool>("fission_example", p_fission_example_, false);

  nh_local_.param<double>("loop_rate", p_loop_rate_, 10.0);
  nh_local_.param<double>("radius_margin", p_radius_margin_, 0.25);

  nh_local_.getParam("x_vector", p_x_vector_);
  nh_local_.getParam("y_vector", p_y_vector_);
  nh_local_.getParam("r_vector", p_r_vector_);

  nh_local_.getParam("vx_vector", p_vx_vector_);
  nh_local_.getParam("vy_vector", p_vy_vector_);

  nh_local_.param<string>("frame_id", p_frame_id_, string("map"));

  p_sampling_time_ = 1.0 / p_loop_rate_;
  timer_.setPeriod(ros::Duration(p_sampling_time_), false);

  if (p_active_ != prev_active) {
    if (p_active_) {
      obstacle_pub_ = nh_.advertise<obstacle_detector::Obstacles>("obstacles", 10);
      timer_.start();
    }
    else {
      obstacle_pub_.shutdown();
      timer_.stop();
    }
  }

  obstacles_.header.frame_id = p_frame_id_;
  obstacles_.circles.clear();

  if (p_x_vector_.size() != p_y_vector_.size() || p_x_vector_.size() != p_r_vector_.size() ||
      p_x_vector_.size() != p_vx_vector_.size() || p_x_vector_.size() != p_vy_vector_.size())
    return false;

  for (int idx = 0; idx < p_x_vector_.size(); ++idx) {
    CircleObstacle circle;
    circle.center.x = p_x_vector_[idx];
    circle.center.y = p_y_vector_[idx];
    circle.radius = p_r_vector_[idx];
    circle.true_radius = p_r_vector_[idx] - p_radius_margin_;;

    circle.velocity.x = p_vx_vector_[idx];
    circle.velocity.y = p_vy_vector_[idx];

    obstacles_.circles.push_back(circle);
  }

  if (p_reset_)
    reset();

  return true;
}

void ObstaclePublisher::timerCallback(const ros::TimerEvent& e) {
  t_ += p_sampling_time_;

  calculateObstaclesPositions(p_sampling_time_);

  if (p_fusion_example_)
    fusionExample(t_);
  else if (p_fission_example_)
    fissionExample(t_);

  if (obstacles_.circles.size() > 0)
    publishObstacles();
}

void ObstaclePublisher::calculateObstaclesPositions(double dt) {
  for (auto& circ : obstacles_.circles) {
    circ.center.x += circ.velocity.x * dt;
    circ.center.y += circ.velocity.y * dt;
  }
}

void ObstaclePublisher::fusionExample(double t) {
  CircleObstacle circ1, circ2;

  obstacles_.circles.clear();

  if (t < 5.0) {
    circ1.center.x = -1.20 + 0.2 * t;
    circ1.center.y = 0.0;
    circ1.radius = 0.20;

    circ2.center.x = 1.20 - 0.2 * t;
    circ2.center.y = 0.0;
    circ2.radius = 0.20;

    obstacles_.circles.push_back(circ1);
    obstacles_.circles.push_back(circ2);
  }
  else if (t < 15.0) {
    circ1.center.x = 0.0;
    circ1.center.y = 0.0;
    circ1.radius = 0.20 + 0.20 * exp(-(t - 5.0) / 1.0);

    obstacles_.circles.push_back(circ1);
  }
  else  if (t > 20.0)
    reset();

  circ1.true_radius = circ1.radius;
  circ2.true_radius = circ2.radius;
}

void ObstaclePublisher::fissionExample(double t) {
  CircleObstacle circ1, circ2;

  obstacles_.circles.clear();

  if (t < 5.0) {
    circ1.center.x = 0.0;
    circ1.center.y = 0.0;
    circ1.radius = 0.20;

    obstacles_.circles.push_back(circ1);
  }
  else if (t < 6.0) {
    circ1.center.x = 0.0;
    circ1.center.y = 0.0;
    circ1.radius = 0.20 + 0.20 * (1.0 - exp(-(t - 5.0) / 1.0));

    obstacles_.circles.push_back(circ1);
  }
  else if (t < 16.0){
    circ1.center.x = -0.20 - 0.2 * (t - 6.0);
    circ1.center.y = 0.0;
    circ1.radius = 0.20;

    circ2.center.x = 0.20 + 0.2 * (t - 6.0);
    circ2.center.y = 0.0;
    circ2.radius = 0.20;

    obstacles_.circles.push_back(circ1);
    obstacles_.circles.push_back(circ2);
  }
  else if (t > 20.0)
    reset();

  circ1.true_radius = circ1.radius;
  circ2.true_radius = circ2.radius;
}

void ObstaclePublisher::publishObstacles() {
  obstacle_detector::ObstaclesPtr obstacles_msg(new obstacle_detector::Obstacles);
  *obstacles_msg = obstacles_;

  obstacles_msg->header.stamp = ros::Time::now();
  obstacle_pub_.publish(obstacles_msg);
}

void ObstaclePublisher::reset() {
  t_ = 0.0;
  p_reset_ = false;
  nh_local_.setParam("reset", false);
}
