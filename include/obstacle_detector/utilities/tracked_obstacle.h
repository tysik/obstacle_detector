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

#include <obstacle_detector/Obstacles.h>
#include "obstacle_detector/utilities/kalman.h"

namespace obstacle_detector
{

class TrackedObstacle {
public:
  TrackedObstacle(const CircleObstacle& obstacle) : obstacle_(obstacle), kf_(0, 3, 5) {
    fade_counter_ = s_fade_counter_size_;
    initKF();
  }

  void predictState() {
    kf_.predictState();

    obstacle_.center.x = kf_.q_pred(0);
    obstacle_.velocity.x = kf_.q_pred(1);

    obstacle_.center.y = kf_.q_pred(2);
    obstacle_.velocity.y = kf_.q_pred(3);

    obstacle_.radius = kf_.q_pred(4);

    fade_counter_--;
  }

  void correctState(const CircleObstacle& new_obstacle) {
    kf_.y(0) = new_obstacle.center.x;
    kf_.y(1) = new_obstacle.center.y;
    kf_.y(2) = new_obstacle.radius;

    kf_.correctState();

    obstacle_.center.x = kf_.q_est(0);
    obstacle_.velocity.x = kf_.q_est(1);

    obstacle_.center.y = kf_.q_est(2);
    obstacle_.velocity.y = kf_.q_est(3);

    obstacle_.radius = kf_.q_est(4);

    fade_counter_ = s_fade_counter_size_;
  }

  void updateState() {
//    kf_.predictState();
//    kf_.correctState(); // ? where are measurements ?

//    obstacle_.center.x = kf_.q_est(0);
//    obstacle_.velocity.x = kf_.q_est(1);

//    obstacle_.center.y = kf_.q_est(2);
//    obstacle_.velocity.y = kf_.q_est(3);

//    obstacle_.radius = kf_.q_est(4);

//    fade_counter_--;
  }

  static void setSamplingTime(double tp) {
    s_sampling_time_ = tp;
  }

  static void setCounterSize(int size) {
    s_fade_counter_size_ = size;
  }

  static void setCovariances(double process_var, double process_rate_var, double measurement_var) {
    s_process_variance_ = process_var;
    s_process_rate_variance_ = process_rate_var;
    s_measurement_variance_ = measurement_var;
  }

  bool hasFaded() const { return ((fade_counter_ <= 0) ? true : false); }
  const CircleObstacle& getObstacle() const { return obstacle_; }
  const KalmanFilter& getKF() const { return kf_; }

private:
  void initKF() {
    kf_.A(0, 1) = s_sampling_time_;
    kf_.A(2, 3) = s_sampling_time_;

    kf_.C(0, 0) = 1.0;
    kf_.C(1, 2) = 1.0;
    kf_.C(2, 4) = 1.0;

    kf_.R *= s_measurement_variance_;

    kf_.Q(0, 0) = s_process_variance_;
    kf_.Q(1, 1) = s_process_rate_variance_;
    kf_.Q(2, 2) = s_process_variance_;
    kf_.Q(3, 3) = s_process_rate_variance_;
    kf_.Q(4, 4) = s_process_variance_;

    kf_.q_est(0) = kf_.q_pred(0) = obstacle_.center.x;
    kf_.q_est(1) = kf_.q_pred(1) = obstacle_.velocity.x;
    kf_.q_est(2) = kf_.q_pred(2) = obstacle_.center.y;
    kf_.q_est(3) = kf_.q_pred(3) = obstacle_.velocity.y;
    kf_.q_est(4) = kf_.q_pred(4) = obstacle_.radius;
  }

  CircleObstacle obstacle_;
  KalmanFilter kf_;
  int fade_counter_;

  // Common variables
  static int s_fade_counter_size_;
  static double s_sampling_time_;
  static double s_process_variance_;
  static double s_process_rate_variance_;
  static double s_measurement_variance_;
};

}
