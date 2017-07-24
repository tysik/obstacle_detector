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

#include "obstacle_detector/panels/scans_merger_panel.h"

using namespace obstacle_detector;
using namespace std;

ScansMergerPanel::ScansMergerPanel(QWidget* parent) : rviz::Panel(parent), nh_(""), nh_local_("scans_merger") {
  params_cli_ = nh_local_.serviceClient<std_srvs::Empty>("params");
  getParams();

  activate_checkbox_ = new QCheckBox("On/Off");
  scan_checkbox_ = new QCheckBox("Publish scan");
  pcl_checkbox_ = new QCheckBox("Publish PCL");

  n_input_ = new QLineEdit();
  r_min_input_ = new QLineEdit();
  r_max_input_ = new QLineEdit();
  x_min_input_ = new QLineEdit();
  x_max_input_ = new QLineEdit();
  y_min_input_ = new QLineEdit();
  y_max_input_ = new QLineEdit();
  fixed_frame_id_input_ = new QLineEdit();
  target_frame_id_input_ = new QLineEdit();

  n_input_->setAlignment(Qt::AlignRight);
  r_min_input_->setAlignment(Qt::AlignRight);
  r_max_input_->setAlignment(Qt::AlignRight);
  x_min_input_->setAlignment(Qt::AlignRight);
  x_max_input_->setAlignment(Qt::AlignRight);
  y_min_input_->setAlignment(Qt::AlignRight);
  y_max_input_->setAlignment(Qt::AlignRight);
  fixed_frame_id_input_->setAlignment(Qt::AlignRight);
  target_frame_id_input_->setAlignment(Qt::AlignRight);

  QFrame* lines[5];
  for (auto& line : lines) {
    line = new QFrame();
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);
  }

  QSpacerItem* margin = new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Fixed);

  QHBoxLayout* scan_pcl_layout = new QHBoxLayout;
  scan_pcl_layout->addItem(margin);
  scan_pcl_layout->addWidget(scan_checkbox_);
  scan_pcl_layout->addItem(margin);
  scan_pcl_layout->addWidget(pcl_checkbox_);
  scan_pcl_layout->addItem(margin);

  QHBoxLayout* beams_num_layout = new QHBoxLayout;
  beams_num_layout->addItem(margin);
  beams_num_layout->addWidget(new QLabel("Number of beams:"));
  beams_num_layout->addWidget(n_input_);
  beams_num_layout->addItem(margin);

  QHBoxLayout* ranges_lim_layout = new QHBoxLayout;
  ranges_lim_layout->addItem(margin);
  ranges_lim_layout->addWidget(new QLabel("r<sub>min</sub>:"), 0, Qt::AlignRight);
  ranges_lim_layout->addWidget(r_min_input_);
  ranges_lim_layout->addWidget(new QLabel("m  "), 0, Qt::AlignLeft);
  ranges_lim_layout->addWidget(new QLabel("r<sub>max</sub>:"), 0, Qt::AlignRight);
  ranges_lim_layout->addWidget(r_max_input_);
  ranges_lim_layout->addWidget(new QLabel("m"), 0, Qt::AlignLeft);
  ranges_lim_layout->addItem(margin);

  QVBoxLayout* beams_layout = new QVBoxLayout;
  beams_layout->addLayout(beams_num_layout);
  beams_layout->addLayout(ranges_lim_layout);

  QGroupBox* scan_box = new QGroupBox("Beams limits:");
  scan_box->setLayout(beams_layout);

  QHBoxLayout* x_lim_layout = new QHBoxLayout;
  x_lim_layout->addItem(margin);
  x_lim_layout->addWidget(new QLabel("x<sub>min</sub>:"), 0, Qt::AlignRight);
  x_lim_layout->addWidget(x_min_input_);
  x_lim_layout->addWidget(new QLabel("m  "), 0, Qt::AlignLeft);
  x_lim_layout->addWidget(new QLabel("x<sub>max</sub>:"), 0, Qt::AlignRight);
  x_lim_layout->addWidget(x_max_input_);
  x_lim_layout->addWidget(new QLabel("m"), 0, Qt::AlignLeft);
  x_lim_layout->addItem(margin);

  QHBoxLayout* y_lim_layout = new QHBoxLayout;
  y_lim_layout->addItem(margin);
  y_lim_layout->addWidget(new QLabel("y<sub>min</sub>:"), 0, Qt::AlignRight);
  y_lim_layout->addWidget(y_min_input_);
  y_lim_layout->addWidget(new QLabel("m  "), 0, Qt::AlignLeft);
  y_lim_layout->addWidget(new QLabel("y<sub>max</sub>:"), 0, Qt::AlignRight);
  y_lim_layout->addWidget(y_max_input_);
  y_lim_layout->addWidget(new QLabel("m"), 0, Qt::AlignLeft);
  y_lim_layout->addItem(margin);

  QVBoxLayout* xy_lim_layout = new QVBoxLayout;
  xy_lim_layout->addLayout(x_lim_layout);
  xy_lim_layout->addLayout(y_lim_layout);

  QGroupBox* xy_lim_box = new QGroupBox("Coordinates limits:");
  xy_lim_box->setLayout(xy_lim_layout);

  QHBoxLayout* fixed_frame_layout = new QHBoxLayout;
  fixed_frame_layout->addItem(margin);
  fixed_frame_layout->addWidget(new QLabel("Fixed frame:"));
  fixed_frame_layout->addWidget(fixed_frame_id_input_, 0, Qt::AlignLeft);
  fixed_frame_layout->addItem(margin);

  QHBoxLayout* target_frame_layout = new QHBoxLayout;
  target_frame_layout->addItem(margin);
  target_frame_layout->addWidget(new QLabel("Target frame:"));
  target_frame_layout->addWidget(target_frame_id_input_, 0, Qt::AlignLeft);
  target_frame_layout->addItem(margin);

  QVBoxLayout* frames_layout = new QVBoxLayout;
  frames_layout->addLayout(fixed_frame_layout);
  frames_layout->addLayout(target_frame_layout);

  QGroupBox* frames_box = new QGroupBox("Frames:");
  frames_box->setLayout(frames_layout);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(activate_checkbox_);
  layout->addWidget(lines[0]);
  layout->addLayout(scan_pcl_layout);
  layout->addWidget(lines[1]);
  layout->addWidget(scan_box);
  layout->addWidget(lines[2]);
  layout->addWidget(xy_lim_box);
  layout->addWidget(lines[3]);
  layout->addWidget(frames_box);
  layout->setAlignment(layout, Qt::AlignCenter);
  setLayout(layout);

  connect(activate_checkbox_, SIGNAL(clicked()), this, SLOT(processInputs()));
  connect(scan_checkbox_, SIGNAL(clicked()), this, SLOT(processInputs()));
  connect(pcl_checkbox_, SIGNAL(clicked()), this, SLOT(processInputs()));

  connect(n_input_, SIGNAL(editingFinished()), this, SLOT(processInputs()));
  connect(r_min_input_, SIGNAL(editingFinished()), this, SLOT(processInputs()));
  connect(r_max_input_, SIGNAL(editingFinished()), this, SLOT(processInputs()));
  connect(x_min_input_, SIGNAL(editingFinished()), this, SLOT(processInputs()));
  connect(x_max_input_, SIGNAL(editingFinished()), this, SLOT(processInputs()));
  connect(y_min_input_, SIGNAL(editingFinished()), this, SLOT(processInputs()));
  connect(y_max_input_, SIGNAL(editingFinished()), this, SLOT(processInputs()));
  connect(fixed_frame_id_input_, SIGNAL(editingFinished()), this, SLOT(processInputs()));
  connect(target_frame_id_input_, SIGNAL(editingFinished()), this, SLOT(processInputs()));

  evaluateParams();
}

void ScansMergerPanel::processInputs() {
  verifyInputs();
  setParams();
  evaluateParams();
  notifyParamsUpdate();
}

void ScansMergerPanel::verifyInputs() {
  p_active_ = activate_checkbox_->isChecked();
  p_publish_scan_ = scan_checkbox_->isChecked();
  p_publish_pcl_ = pcl_checkbox_->isChecked();

  try { p_ranges_num_ = boost::lexical_cast<int>(n_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_ranges_num_ = 0; n_input_->setText("0"); }

  try { p_min_scanner_range_ = boost::lexical_cast<double>(r_min_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_min_scanner_range_ = 0.0; r_min_input_->setText("0.0"); }

  try { p_max_scanner_range_ = boost::lexical_cast<double>(r_max_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_max_scanner_range_ = 0.0; r_max_input_->setText("0.0"); }

  try { p_min_x_range_ = boost::lexical_cast<double>(x_min_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_min_x_range_ = 0.0; x_min_input_->setText("0.0"); }

  try { p_max_x_range_ = boost::lexical_cast<double>(x_max_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_max_x_range_ = 0.0; x_max_input_->setText("0.0"); }

  try { p_min_y_range_ = boost::lexical_cast<double>(y_min_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_min_y_range_ = 0.0; y_min_input_->setText("0.0"); }

  try { p_max_y_range_ = boost::lexical_cast<double>(y_max_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_max_y_range_ = 0.0; y_max_input_->setText("0.0"); }

  p_fixed_frame_id_ = fixed_frame_id_input_->text().toStdString();
  p_target_frame_id_ = target_frame_id_input_->text().toStdString();
}

void ScansMergerPanel::setParams() {
  nh_local_.setParam("active", p_active_);
  nh_local_.setParam("publish_scan", p_publish_scan_);
  nh_local_.setParam("publish_pcl", p_publish_pcl_);

  nh_local_.setParam("ranges_num", p_ranges_num_);

  nh_local_.setParam("min_scanner_range", p_min_scanner_range_);
  nh_local_.setParam("max_scanner_range", p_max_scanner_range_);

  nh_local_.setParam("min_x_range", p_min_x_range_);
  nh_local_.setParam("max_x_range", p_max_x_range_);
  nh_local_.setParam("min_y_range", p_min_y_range_);
  nh_local_.setParam("max_y_range", p_max_y_range_);

  nh_local_.setParam("fixed_frame_id", p_fixed_frame_id_);
  nh_local_.setParam("target_frame_id", p_target_frame_id_);
}

void ScansMergerPanel::getParams() {
  p_active_ = nh_local_.param("active", false);
  p_publish_scan_ = nh_local_.param("publish_scan", false);
  p_publish_pcl_ = nh_local_.param("publish_pcl", false);

  p_ranges_num_ = nh_local_.param("ranges_num", 0);

  p_min_scanner_range_ = nh_local_.param("min_scanner_range", 0.0);
  p_max_scanner_range_ = nh_local_.param("max_scanner_range", 0.0);

  p_min_x_range_ = nh_local_.param("min_x_range", 0.0);
  p_max_x_range_ = nh_local_.param("max_x_range", 0.0);
  p_min_y_range_ = nh_local_.param("min_y_range", 0.0);
  p_max_y_range_ = nh_local_.param("max_y_range", 0.0);

  p_fixed_frame_id_ = nh_local_.param("fixed_frame_id", std::string(""));
  p_target_frame_id_ = nh_local_.param("target_frame_id", std::string(""));
}

void ScansMergerPanel::evaluateParams() {
  activate_checkbox_->setChecked(p_active_);

  scan_checkbox_->setEnabled(p_active_);
  pcl_checkbox_->setEnabled(p_active_);

  scan_checkbox_->setChecked(p_publish_scan_);
  pcl_checkbox_->setChecked(p_publish_pcl_);

  n_input_->setEnabled(p_active_);
  r_min_input_->setEnabled(p_active_);
  r_max_input_->setEnabled(p_active_);
  x_min_input_->setEnabled(p_active_);
  x_max_input_->setEnabled(p_active_);
  y_min_input_->setEnabled(p_active_);
  y_max_input_->setEnabled(p_active_);
  fixed_frame_id_input_->setEnabled(p_active_);
  target_frame_id_input_->setEnabled(p_active_);

  n_input_->setText(QString::number(p_ranges_num_));
  r_min_input_->setText(QString::number(p_min_scanner_range_));
  r_max_input_->setText(QString::number(p_max_scanner_range_));
  x_min_input_->setText(QString::number(p_min_x_range_));
  x_max_input_->setText(QString::number(p_max_x_range_));
  y_min_input_->setText(QString::number(p_min_y_range_));
  y_max_input_->setText(QString::number(p_max_y_range_));

  fixed_frame_id_input_->setText(QString::fromStdString(p_fixed_frame_id_));
  target_frame_id_input_->setText(QString::fromStdString(p_target_frame_id_));
}

void ScansMergerPanel::notifyParamsUpdate() {
  std_srvs::Empty empty;
  if (!params_cli_.call(empty)) {
    p_active_ = false;
    setParams();
    evaluateParams();
  }
}

void ScansMergerPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
}

void ScansMergerPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(obstacle_detector::ScansMergerPanel, rviz::Panel)
