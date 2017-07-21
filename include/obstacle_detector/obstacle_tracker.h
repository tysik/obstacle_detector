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

#include <list>
#include <string>
#include <ros/ros.h>
#include <armadillo>
#include <std_srvs/Empty.h>
#include <obstacle_detector/Obstacles.h>

#include "obstacle_detector/utilities/tracked_obstacle.h"
#include "obstacle_detector/utilities/math_utilities.h"

namespace obstacle_detector
{

class ObstacleTracker {
public:
  ObstacleTracker(ros::NodeHandle& nh, ros::NodeHandle& nh_local);
  ~ObstacleTracker();

private:
  bool updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  void timerCallback(const ros::TimerEvent&);
  void obstaclesCallback(const obstacle_detector::Obstacles::ConstPtr new_obstacles);

  void initialize() { std_srvs::Empty empt; updateParams(empt.request, empt.response); }

  double obstacleCostFunction(const CircleObstacle& new_obstacle, const CircleObstacle& old_obstacle);
  void calculateCostMatrix(const std::vector<CircleObstacle>& new_obstacles, arma::mat& cost_matrix);
  void calculateRowMinIndices(const arma::mat& cost_matrix, std::vector<int>& row_min_indices);
  void calculateColMinIndices(const arma::mat& cost_matrix, std::vector<int>& col_min_indices);

  bool fusionObstacleUsed(const int idx, const std::vector<int>& col_min_indices, const std::vector<int>& used_new, const std::vector<int>& used_old);
  bool fusionObstaclesCorrespond(const int idx, const int jdx, const std::vector<int>& col_min_indices, const std::vector<int>& used_old);
  bool fissionObstacleUsed(const int idx, const int T, const std::vector<int>& row_min_indices, const std::vector<int>& used_new, const std::vector<int>& used_old);
  bool fissionObstaclesCorrespond(const int idx, const int jdx, const std::vector<int>& row_min_indices, const std::vector<int>& used_new);

  void fuseObstacles(const std::vector<int>& fusion_indices, const std::vector<int>& col_min_indices,
                     std::vector<TrackedObstacle>& new_tracked, const Obstacles::ConstPtr& new_obstacles);
  void fissureObstacle(const std::vector<int>& fission_indices, const std::vector<int>& row_min_indices,
                       std::vector<TrackedObstacle>& new_tracked, const Obstacles::ConstPtr& new_obstacles);

  void updateObstacles();
  void publishObstacles();

  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;

  ros::Subscriber obstacles_sub_;
  ros::Publisher obstacles_pub_;
  ros::ServiceServer params_srv_;
  ros::Timer timer_;

  double radius_margin_;
  obstacle_detector::Obstacles obstacles_;

  std::vector<TrackedObstacle> tracked_obstacles_;
  std::vector<CircleObstacle> untracked_obstacles_;

  // Parameters
  bool p_active_;
  bool p_copy_segments_;

  double p_tracking_duration_;
  double p_loop_rate_;
  double p_sampling_time_;
  double p_sensor_rate_;
  double p_min_correspondence_cost_;
  double p_std_correspondence_dev_;
  double p_process_variance_;
  double p_process_rate_variance_;
  double p_measurement_variance_;

  std::string p_frame_id_;
};

} // namespace obstacle_detector
