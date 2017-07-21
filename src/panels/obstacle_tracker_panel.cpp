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

#include "obstacle_detector/panels/obstacle_tracker_panel.h"

using namespace obstacle_detector;
using namespace std;

ObstacleTrackerPanel::ObstacleTrackerPanel(QWidget* parent) : rviz::Panel(parent), nh_(""), nh_local_("obstacle_tracker") {
  params_cli_ = nh_local_.serviceClient<std_srvs::Empty>("params");
  getParams();

  activate_checkbox_ = new QCheckBox("On/Off");

  tracking_duration_input_ = new QLineEdit();
  loop_rate_input_ = new QLineEdit();
  min_corr_cost_input_ = new QLineEdit();
  std_corr_dev_input_ = new QLineEdit();
  process_var_input_ = new QLineEdit();
  process_rate_var_input_ = new QLineEdit();
  measure_var_input_ = new QLineEdit();

  tracking_duration_input_->setAlignment(Qt::AlignRight);
  loop_rate_input_->setAlignment(Qt::AlignRight);
  min_corr_cost_input_->setAlignment(Qt::AlignRight);
  std_corr_dev_input_->setAlignment(Qt::AlignRight);
  process_var_input_->setAlignment(Qt::AlignRight);
  process_rate_var_input_->setAlignment(Qt::AlignRight);
  measure_var_input_->setAlignment(Qt::AlignRight);

  QSpacerItem* margin = new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Fixed);

  QFrame* lines[4];
  for (auto& line : lines) {
    line = new QFrame();
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);
  }

  QGroupBox* tracking_box = new QGroupBox("General:");
  QGridLayout* tracking_layout = new QGridLayout;
  tracking_layout->addWidget(new QLabel("Tracking rate:"), 0, 0, Qt::AlignRight);
  tracking_layout->addWidget(loop_rate_input_, 0, 1, Qt::AlignLeft);
  tracking_layout->addWidget(new QLabel("Hz"), 0, 2, Qt::AlignLeft);
  tracking_layout->addWidget(new QLabel("Tracking duration:"), 1, 0, Qt::AlignRight);
  tracking_layout->addWidget(tracking_duration_input_, 1, 1, Qt::AlignLeft);
  tracking_layout->addWidget(new QLabel("s"), 1, 2, Qt::AlignLeft);
  tracking_box->setLayout(tracking_layout);

  QGroupBox* corr_box = new QGroupBox("Correspondence:");
  QGridLayout* corr_layout = new QGridLayout;
  corr_layout->addWidget(new QLabel("Minimal cost:"), 0, 0, Qt::AlignRight);
  corr_layout->addWidget(min_corr_cost_input_, 0, 1, Qt::AlignLeft);
  corr_layout->addWidget(new QLabel("m"), 0, 2, Qt::AlignLeft);
  corr_layout->addWidget(new QLabel("Standard deviation:"), 1, 0, Qt::AlignRight);
  corr_layout->addWidget(std_corr_dev_input_, 1, 1, Qt::AlignLeft);
  corr_layout->addWidget(new QLabel("m"), 1, 2, Qt::AlignLeft);
  corr_box->setLayout(corr_layout);

  QGroupBox* kf_box = new QGroupBox("Kalman Filter:");
  QGridLayout* kf_layout = new QGridLayout;
  kf_layout->addWidget(new QLabel("Process variance:"), 0, 0, Qt::AlignRight);
  kf_layout->addWidget(process_var_input_, 0, 1, Qt::AlignLeft);
  kf_layout->addWidget(new QLabel("m<sup>2</sup>"), 0, 2, Qt::AlignLeft);
  kf_layout->addWidget(new QLabel("Process rate variance:"), 1, 0, Qt::AlignRight);
  kf_layout->addWidget(process_rate_var_input_, 1, 1, Qt::AlignLeft);
  kf_layout->addWidget(new QLabel("(m/s)<sup>2</sup>"), 1, 2, Qt::AlignLeft);
  kf_layout->addWidget(new QLabel("Measurement variance:"), 2, 0, Qt::AlignRight);
  kf_layout->addWidget(measure_var_input_, 2, 1, Qt::AlignLeft);
  kf_layout->addWidget(new QLabel("m<sup>2</sup>"), 2, 2, Qt::AlignLeft);
  kf_box->setLayout(kf_layout);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(activate_checkbox_);
  layout->addWidget(lines[0]);
  layout->addWidget(tracking_box);
  layout->addWidget(lines[1]);
  layout->addWidget(corr_box);
  layout->addWidget(lines[2]);
  layout->addWidget(kf_box);
  layout->setAlignment(layout, Qt::AlignCenter);
  setLayout(layout);

  connect(activate_checkbox_, SIGNAL(clicked()), this, SLOT(processInputs()));

  connect(tracking_duration_input_, SIGNAL(editingFinished()), this, SLOT(processInputs()));
  connect(loop_rate_input_, SIGNAL(editingFinished()), this, SLOT(processInputs()));
  connect(min_corr_cost_input_, SIGNAL(editingFinished()), this, SLOT(processInputs()));
  connect(std_corr_dev_input_, SIGNAL(editingFinished()), this, SLOT(processInputs()));
  connect(process_var_input_, SIGNAL(editingFinished()), this, SLOT(processInputs()));
  connect(process_rate_var_input_, SIGNAL(editingFinished()), this, SLOT(processInputs()));
  connect(measure_var_input_, SIGNAL(editingFinished()), this, SLOT(processInputs()));

  evaluateParams();
}

void ObstacleTrackerPanel::processInputs() {
  verifyInputs();
  setParams();
  evaluateParams();
  notifyParamsUpdate();
}

void ObstacleTrackerPanel::verifyInputs() {
  p_active_ = activate_checkbox_->isChecked();

  try { p_loop_rate_ = boost::lexical_cast<double>(loop_rate_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_loop_rate_ = 1.0; loop_rate_input_->setText("1.0"); }

  try { p_tracking_duration_ = boost::lexical_cast<double>(tracking_duration_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_tracking_duration_ = 0.0; tracking_duration_input_->setText("0.0"); }

  try { p_min_correspondence_cost_ = boost::lexical_cast<double>(min_corr_cost_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_min_correspondence_cost_ = 0.0; min_corr_cost_input_->setText("0.0"); }

  try { p_std_correspondence_dev_ = boost::lexical_cast<double>(std_corr_dev_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_std_correspondence_dev_ = 0.0; std_corr_dev_input_->setText("0.0"); }

  try { p_process_variance_ = boost::lexical_cast<double>(process_var_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_process_variance_ = 0.0; process_var_input_->setText("0.0"); }

  try { p_process_rate_variance_ = boost::lexical_cast<double>(process_rate_var_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_process_rate_variance_ = 0.0; process_rate_var_input_->setText("0.0"); }

  try { p_measurement_variance_ = boost::lexical_cast<double>(measure_var_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_measurement_variance_ = 0.0; measure_var_input_->setText("0.0"); }
}

void ObstacleTrackerPanel::setParams() {
  nh_local_.setParam("active", p_active_);

  nh_local_.setParam("loop_rate", p_loop_rate_);
  nh_local_.setParam("tracking_duration", p_tracking_duration_);
  nh_local_.setParam("min_correspondence_cost", p_min_correspondence_cost_);
  nh_local_.setParam("std_correspondence_dev", p_std_correspondence_dev_);
  nh_local_.setParam("process_variance", p_process_variance_);
  nh_local_.setParam("process_rate_variance", p_process_rate_variance_);
  nh_local_.setParam("measurement_variance", p_measurement_variance_);
}

void ObstacleTrackerPanel::getParams() {
  p_active_ = nh_local_.param("active", false);

  p_loop_rate_ = nh_local_.param("loop_rate", 0.0);
  p_tracking_duration_ = nh_local_.param("tracking_duration", 0.0);
  p_min_correspondence_cost_ = nh_local_.param("min_correspondence_cost", 0.0);
  p_std_correspondence_dev_ = nh_local_.param("std_correspondence_dev", 0.0);
  p_process_variance_ = nh_local_.param("process_variance", 0.0);
  p_process_rate_variance_ = nh_local_.param("process_rate_variance", 0.0);
  p_measurement_variance_ = nh_local_.param("measurement_variance", 0.0);
}

void ObstacleTrackerPanel::evaluateParams() {
  activate_checkbox_->setChecked(p_active_);

  loop_rate_input_->setEnabled(p_active_);
  loop_rate_input_->setText(QString::number(p_loop_rate_));

  tracking_duration_input_->setEnabled(p_active_);
  tracking_duration_input_->setText(QString::number(p_tracking_duration_));

  min_corr_cost_input_->setEnabled(p_active_);
  min_corr_cost_input_->setText(QString::number(p_min_correspondence_cost_));

  std_corr_dev_input_->setEnabled(p_active_);
  std_corr_dev_input_->setText(QString::number(p_std_correspondence_dev_));

  process_var_input_->setEnabled(p_active_);
  process_var_input_->setText(QString::number(p_process_variance_));

  process_rate_var_input_->setEnabled(p_active_);
  process_rate_var_input_->setText(QString::number(p_process_rate_variance_));

  measure_var_input_->setEnabled(p_active_);
  measure_var_input_->setText(QString::number(p_measurement_variance_));
}

void ObstacleTrackerPanel::notifyParamsUpdate() {
  std_srvs::Empty empty;
  if (!params_cli_.call(empty)) {
    p_active_ = false;
    setParams();
    evaluateParams();
  }
}

void ObstacleTrackerPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
}

void ObstacleTrackerPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(obstacle_detector::ObstacleTrackerPanel, rviz::Panel)
