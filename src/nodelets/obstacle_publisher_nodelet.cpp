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

#include <memory>
#include <nodelet/nodelet.h>

#include "obstacle_detector/obstacle_publisher.h"

namespace obstacle_detector
{

class ObstaclePublisherNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit() {
    ros::NodeHandle nh = getNodeHandle();
    ros::NodeHandle nh_local = getPrivateNodeHandle();

    try {
    NODELET_INFO("[Obstacle Publisher]: Initializing nodelet");
    obstacle_publisher_ = std::shared_ptr<ObstaclePublisher>(new ObstaclePublisher(nh, nh_local));
    }
    catch (const char* s) {
      NODELET_FATAL_STREAM("[Obstacle Publisher]: " << s);
    }
    catch (...) {
      NODELET_FATAL_STREAM("[Obstacle Publisher]: Unexpected error");
    }
  }

  virtual ~ObstaclePublisherNodelet() {
    NODELET_INFO("[Obstacle Publisher]: Shutdown");
  }

private:
  std::shared_ptr<ObstaclePublisher> obstacle_publisher_;
};

} // namespace obstacle_detector

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(obstacle_detector::ObstaclePublisherNodelet, nodelet::Nodelet)
