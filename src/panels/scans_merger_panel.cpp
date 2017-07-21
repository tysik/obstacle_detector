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
  frame_id_input_ = new QLineEdit();

  n_input_->setAlignment(Qt::AlignRight);
  r_min_input_->setAlignment(Qt::AlignRight);
  r_max_input_->setAlignment(Qt::AlignRight);
  x_min_input_->setAlignment(Qt::AlignRight);
  x_max_input_->setAlignment(Qt::AlignRight);
  y_min_input_->setAlignment(Qt::AlignRight);
  y_max_input_->setAlignment(Qt::AlignRight);
  frame_id_input_->setAlignment(Qt::AlignRight);

  QFrame* lines[5];
  for (auto& line : lines) {
    line = new QFrame();
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);
  }

  QSpacerItem* margin = new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Fixed);

  QHBoxLayout* layout_1 = new QHBoxLayout;
  layout_1->addItem(margin);
  layout_1->addWidget(scan_checkbox_);
  layout_1->addItem(margin);
  layout_1->addWidget(pcl_checkbox_);
  layout_1->addItem(margin);

  QHBoxLayout* layout_2 = new QHBoxLayout;
  layout_2->addItem(margin);
  layout_2->addWidget(new QLabel("Number of ranges:"));
  layout_2->addWidget(n_input_);
  layout_2->addItem(margin);

  QHBoxLayout* layout_3 = new QHBoxLayout;
  layout_3->addItem(margin);
  layout_3->addWidget(new QLabel("r<sub>min</sub>:"), 0, Qt::AlignRight);
  layout_3->addWidget(r_min_input_);
  layout_3->addWidget(new QLabel("m  "), 0, Qt::AlignLeft);
  layout_3->addWidget(new QLabel("r<sub>max</sub>:"), 0, Qt::AlignRight);
  layout_3->addWidget(r_max_input_);
  layout_3->addWidget(new QLabel("m"), 0, Qt::AlignLeft);
  layout_3->addItem(margin);

  QVBoxLayout* ranges_layout = new QVBoxLayout;
  ranges_layout->addLayout(layout_2);
  ranges_layout->addLayout(layout_3);

  QGroupBox* scan_box = new QGroupBox("Ranges limits:");
  scan_box->setLayout(ranges_layout);

  QHBoxLayout* layout_4 = new QHBoxLayout;
  layout_4->addItem(margin);
  layout_4->addWidget(new QLabel("x<sub>min</sub>:"), 0, Qt::AlignRight);
  layout_4->addWidget(x_min_input_);
  layout_4->addWidget(new QLabel("m  "), 0, Qt::AlignLeft);
  layout_4->addWidget(new QLabel("x<sub>max</sub>:"), 0, Qt::AlignRight);
  layout_4->addWidget(x_max_input_);
  layout_4->addWidget(new QLabel("m"), 0, Qt::AlignLeft);
  layout_4->addItem(margin);

  QHBoxLayout* layout_5 = new QHBoxLayout;
  layout_5->addItem(margin);
  layout_5->addWidget(new QLabel("y<sub>min</sub>:"), 0, Qt::AlignRight);
  layout_5->addWidget(y_min_input_);
  layout_5->addWidget(new QLabel("m  "), 0, Qt::AlignLeft);
  layout_5->addWidget(new QLabel("y<sub>max</sub>:"), 0, Qt::AlignRight);
  layout_5->addWidget(y_max_input_);
  layout_5->addWidget(new QLabel("m"), 0, Qt::AlignLeft);
  layout_5->addItem(margin);

  QVBoxLayout* coords_layout = new QVBoxLayout;
  coords_layout->addLayout(layout_4);
  coords_layout->addLayout(layout_5);

  QGroupBox* coords_box = new QGroupBox("Coordinates limits:");
  coords_box->setLayout(coords_layout);

  QHBoxLayout* layout_6 = new QHBoxLayout;
  layout_6->addItem(margin);
  layout_6->addWidget(new QLabel("Frame ID:"));
  layout_6->addWidget(frame_id_input_, 0, Qt::AlignLeft);
  layout_6->addItem(margin);

  QGroupBox* frame_box = new QGroupBox("Frames:");
  frame_box->setLayout(layout_6);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(activate_checkbox_);
  layout->addWidget(lines[0]);
  layout->addLayout(layout_1);
  layout->addWidget(lines[1]);
  layout->addWidget(scan_box);
  layout->addWidget(lines[2]);
  layout->addWidget(coords_box);
  layout->addWidget(lines[3]);
  layout->addWidget(frame_box);
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
  connect(frame_id_input_, SIGNAL(editingFinished()), this, SLOT(processInputs()));

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

  p_target_frame_id_ = frame_id_input_->text().toStdString();
}

void ScansMergerPanel::setParams() {
  nh_local_.setParam("ranges_num", p_ranges_num_);

  nh_local_.setParam("active", p_active_);
  nh_local_.setParam("publish_scan", p_publish_scan_);
  nh_local_.setParam("publish_pcl", p_publish_pcl_);

  nh_local_.setParam("min_scanner_range", p_min_scanner_range_);
  nh_local_.setParam("max_scanner_range", p_max_scanner_range_);
  nh_local_.setParam("max_x_range", p_max_x_range_);
  nh_local_.setParam("min_x_range", p_min_x_range_);
  nh_local_.setParam("max_y_range", p_max_y_range_);
  nh_local_.setParam("min_y_range", p_min_y_range_);

  nh_local_.setParam("target_frame_id", p_target_frame_id_);
}

void ScansMergerPanel::getParams() {
  if (!nh_local_.getParam("active", p_active_))
    p_active_ = false;

  if (!nh_local_.getParam("publish_scan", p_publish_scan_))
    p_publish_scan_ = false;

  if (!nh_local_.getParam("publish_pcl", p_publish_pcl_))
    p_publish_pcl_ = false;

  if (!nh_local_.getParam("ranges_num", p_ranges_num_))
    p_ranges_num_ = 0;

  if (!nh_local_.getParam("min_scanner_range", p_min_scanner_range_))
    p_min_scanner_range_ = 0.0;

  if (!nh_local_.getParam("max_scanner_range", p_max_scanner_range_))
    p_max_scanner_range_ = 0.0;

  if (!nh_local_.getParam("max_x_range", p_max_x_range_))
    p_max_x_range_ = 0.0;

  if (!nh_local_.getParam("min_x_range", p_min_x_range_))
    p_min_x_range_ = 0.0;

  if (!nh_local_.getParam("max_y_range", p_max_y_range_))
    p_max_y_range_ = 0.0;

  if (!nh_local_.getParam("min_y_range", p_min_y_range_))
    p_min_y_range_ = 0.0;

  if (!nh_local_.getParam("target_frame_id", p_target_frame_id_))
    p_target_frame_id_ = std::string("-");
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
  frame_id_input_->setEnabled(p_active_);

  n_input_->setText(QString::number(p_ranges_num_));
  r_min_input_->setText(QString::number(p_min_scanner_range_));
  r_max_input_->setText(QString::number(p_max_scanner_range_));
  x_min_input_->setText(QString::number(p_min_x_range_));
  x_max_input_->setText(QString::number(p_max_x_range_));
  y_min_input_->setText(QString::number(p_min_y_range_));
  y_max_input_->setText(QString::number(p_max_y_range_));

  frame_id_input_->setText(QString::fromStdString(p_target_frame_id_));
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
