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

#include "obstacle_detector/displays/obstacles_display.h"

namespace obstacles_display
{

ObstaclesDisplay::ObstaclesDisplay() {
  circle_color_property_ = new rviz::ColorProperty("Circles color", QColor(170, 0, 0), "Color of circles.", this, SLOT(updateCircleColor()));
  margin_color_property_ = new rviz::ColorProperty("Margin color", QColor(0, 170, 0), "Color of margin added around circles.", this, SLOT(updateCircleColor()));
  segment_color_property_ = new rviz::ColorProperty("Segments color", QColor(170, 170, 0), "Color of segments.", this, SLOT(updateSegmentColor()));
  alpha_property_ = new rviz::FloatProperty("Opacity", 0.75, "Value 0,0 is fully transparent, 1,0 is fully opaque.", this, SLOT(updateAlpha()));
  thickness_property_ = new rviz::FloatProperty("Segments thickness", 0.03f, "Width of the segments in meters.", this, SLOT(updateThickness()));
}

void ObstaclesDisplay::onInitialize() {
  MessageFilterDisplay<obstacle_detector::Obstacles>::onInitialize();
}

ObstaclesDisplay::~ObstaclesDisplay() {}

void ObstaclesDisplay::reset() {
  MessageFilterDisplay<obstacle_detector::Obstacles>::reset();
  circle_visuals_.clear();
  segment_visuals_.clear();
}

void ObstaclesDisplay::updateCircleColor() {
  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue main_color = circle_color_property_->getOgreColor();
  Ogre::ColourValue margin_color = margin_color_property_->getOgreColor();

  for (auto& c : circle_visuals_) {
    c->setMainColor(main_color.r, main_color.g, main_color.b, alpha);
    c->setMarginColor(margin_color.r, margin_color.g, margin_color.b, alpha);
  }
}

void ObstaclesDisplay::updateSegmentColor() {
  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = segment_color_property_->getOgreColor();

  for (auto& s : segment_visuals_)
    s->setColor(color.r, color.g, color.b, alpha);
}

void ObstaclesDisplay::updateAlpha() {
  updateCircleColor();
  updateSegmentColor();
}

void ObstaclesDisplay::updateThickness() {
  float width = thickness_property_->getFloat();

  for (auto& s : segment_visuals_)
    s->setWidth(width);
}

void ObstaclesDisplay::processMessage(const obstacle_detector::Obstacles::ConstPtr& obstacles_msg) {
  circle_visuals_.clear();
  segment_visuals_.clear();

  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(obstacles_msg->header.frame_id, obstacles_msg->header.stamp, position, orientation)) {
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'", obstacles_msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
    return;
  }

  for (const auto& circle : obstacles_msg->circles) {
    boost::shared_ptr<CircleVisual> visual;
    visual.reset(new CircleVisual(context_->getSceneManager(), scene_node_));

    visual->setData(circle);
    visual->setFramePosition(position);
    visual->setFrameOrientation(orientation);

    circle_visuals_.push_back(visual);
  }

  for (const auto& segment : obstacles_msg->segments) {
    boost::shared_ptr<SegmentVisual> visual;
    visual.reset(new SegmentVisual(context_->getSceneManager(), scene_node_));

    visual->setData(segment);
    visual->setFramePosition(position);
    visual->setFrameOrientation(orientation);

    segment_visuals_.push_back(visual);
  }

  updateAlpha();
  updateThickness();
}

} // end namespace obstacles_display

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(obstacles_display::ObstaclesDisplay, rviz::Display)

