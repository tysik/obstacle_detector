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

#ifndef Q_MOC_RUN
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <obstacle_detector/Obstacles.h>
#include <tf/transform_listener.h>

#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/message_filter_display.h>
#include <rviz/visualization_manager.h>
#include <rviz/frame_manager.h>

#include "obstacle_detector/displays/circle_visual.h"
#include "obstacle_detector/displays/segment_visual.h"
#endif

namespace obstacles_display
{

class ObstaclesDisplay: public rviz::MessageFilterDisplay<obstacle_detector::Obstacles>
{
Q_OBJECT
public:
  ObstaclesDisplay();
  virtual ~ObstaclesDisplay();

protected:
  virtual void onInitialize();
  virtual void reset();

private Q_SLOTS:
  void updateCircleColor();
  void updateSegmentColor();
  void updateAlpha();
  void updateThickness();

private:
  void processMessage(const obstacle_detector::Obstacles::ConstPtr& obstacles_msg);

  std::vector< boost::shared_ptr<CircleVisual> > circle_visuals_;
  std::vector< boost::shared_ptr<SegmentVisual> > segment_visuals_;

  rviz::ColorProperty* circle_color_property_;
  rviz::ColorProperty* margin_color_property_;
  rviz::ColorProperty* segment_color_property_;
  rviz::FloatProperty* alpha_property_;
  rviz::FloatProperty* thickness_property_;
};

} // end namespace obstacles_display
