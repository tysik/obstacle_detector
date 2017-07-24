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

#include <ros/ros.h>
#include <rviz/panel.h>
#include <std_srvs/Empty.h>

#include <QLabel>
#include <QFrame>
#include <QCheckBox>
#include <QLineEdit>
#include <QPushButton>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>

namespace obstacle_detector
{

class ObstacleExtractorPanel : public rviz::Panel
{
Q_OBJECT
public:
  ObstacleExtractorPanel(QWidget* parent = 0);

  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

private Q_SLOTS:
  void processInputs();

private:
  void verifyInputs();
  void setParams();
  void getParams();
  void evaluateParams();
  void notifyParamsUpdate();

private:
  QCheckBox* activate_checkbox_;
  QCheckBox* use_scan_checkbox_;
  QCheckBox* use_pcl_checkbox_;
  QCheckBox* use_split_merge_checkbox_;
  QCheckBox* circ_from_visib_checkbox_;
  QCheckBox* discard_segments_checkbox_;
  QCheckBox* transform_coords_checkbox_;

  QLineEdit* min_n_input_;
  QLineEdit* dist_prop_input_;
  QLineEdit* group_dist_input_;
  QLineEdit* split_dist_input_;
  QLineEdit* merge_sep_input_;
  QLineEdit* merge_spread_input_;
  QLineEdit* max_radius_input_;
  QLineEdit* radius_enl_input_;
  QLineEdit* frame_id_input_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;

  ros::ServiceClient params_cli_;

  // Parameters
  bool p_active_;
  bool p_use_scan_;
  bool p_use_pcl_;

  bool p_use_split_and_merge_;
  bool p_circles_from_visibles_;
  bool p_discard_converted_segments_;
  bool p_transform_coordinates_;

  int p_min_group_points_;

  double p_distance_proportion_;
  double p_max_group_distance_;

  double p_max_split_distance_;
  double p_max_merge_separation_;
  double p_max_merge_spread_;
  double p_max_circle_radius_;
  double p_radius_enlargement_;

  std::string p_frame_id_;
};

} // namespace obstacle_detector
