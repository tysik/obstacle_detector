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

#include "obstacle_detector/panels/obstacle_publisher_panel.h"

using namespace obstacle_detector;
using namespace std;

ObstaclePublisherPanel::ObstaclePublisherPanel(QWidget* parent) : rviz::Panel(parent), nh_(""), nh_local_("obstacle_publisher") {
  params_cli_ = nh_local_.serviceClient<std_srvs::Empty>("params");
  getParams();

  activate_checkbox_ = new QCheckBox("On/Off");
  fusion_example_checkbox_ = new QCheckBox("Fusion");
  fission_example_checkbox_ = new QCheckBox("Fission");

  obstacles_list_ = new QListWidget();
  add_button_ = new QPushButton("Add obstacle");
  remove_button_ = new QPushButton("Remove selected");
  reset_button_ = new QPushButton("Reset time");

  x_input_ = new QLineEdit();
  y_input_ = new QLineEdit();
  r_input_ = new QLineEdit();
  vx_input_ = new QLineEdit();
  vy_input_ = new QLineEdit();
  frame_input_ = new QLineEdit();

  obstacles_list_->setSelectionMode(QAbstractItemView::MultiSelection);
  frame_input_->setAlignment(Qt::AlignRight);

  QFrame* lines[5];
  for (auto& line : lines) {
    line = new QFrame();
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);
  }

  QSpacerItem* margin = new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Fixed);

  QHBoxLayout* demos_layout = new QHBoxLayout;
  demos_layout->addWidget(fusion_example_checkbox_);
  demos_layout->addWidget(fission_example_checkbox_);

  QGroupBox* demos_box = new QGroupBox("Demos:");
  demos_box->setLayout(demos_layout);

  QHBoxLayout* target_frame_layout = new QHBoxLayout;
  target_frame_layout->addItem(margin);
  target_frame_layout->addWidget(new QLabel("Target frame:"));
  target_frame_layout->addWidget(frame_input_, 0, Qt::AlignLeft);
  target_frame_layout->addItem(margin);

  QGroupBox* frames_box = new QGroupBox("Frames:");
  frames_box->setLayout(target_frame_layout);

  QHBoxLayout* xyr_layout = new QHBoxLayout;
  xyr_layout->addItem(margin);
  xyr_layout->addWidget(new QLabel("x:"));
  xyr_layout->addWidget(x_input_);
  xyr_layout->addWidget(new QLabel("m, "));
  xyr_layout->addWidget(new QLabel("y:"));
  xyr_layout->addWidget(y_input_);
  xyr_layout->addWidget(new QLabel("m, "));
  xyr_layout->addWidget(new QLabel("r:"));
  xyr_layout->addWidget(r_input_);
  xyr_layout->addWidget(new QLabel("m"));
  xyr_layout->addItem(margin);

  QHBoxLayout* vxvy_layout = new QHBoxLayout;
  vxvy_layout->addItem(margin);
  vxvy_layout->addWidget(new QLabel("v<sub>x</sub>:"));
  vxvy_layout->addWidget(vx_input_);
  vxvy_layout->addWidget(new QLabel("m/s, "));
  vxvy_layout->addWidget(new QLabel("v<sub>y</sub>:"));
  vxvy_layout->addWidget(vy_input_);
  vxvy_layout->addWidget(new QLabel("m/s"));
  vxvy_layout->addItem(margin);

  QVBoxLayout* obst_params_layout = new QVBoxLayout;
  obst_params_layout->addWidget(obstacles_list_);
  obst_params_layout->addWidget(remove_button_);
  obst_params_layout->addWidget(lines[2]);
  obst_params_layout->addLayout(xyr_layout);
  obst_params_layout->addLayout(vxvy_layout);
  obst_params_layout->addWidget(add_button_, Qt::AlignCenter);
  obst_params_layout->addWidget(lines[3]);
  obst_params_layout->addWidget(reset_button_, Qt::AlignCenter);

  QGroupBox* obst_box = new QGroupBox("Obstacles:");
  obst_box->setLayout(obst_params_layout);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(activate_checkbox_);
  layout->addWidget(lines[0]);
  layout->addWidget(demos_box);
  layout->addWidget(lines[1]);
  layout->addWidget(frames_box);
  layout->addWidget(lines[4]);
  layout->addWidget(obst_box);
  layout->setAlignment(layout, Qt::AlignCenter);
  setLayout(layout);

  connect(activate_checkbox_, SIGNAL(clicked()), this, SLOT(processInputs()));
  connect(fusion_example_checkbox_, SIGNAL(clicked()), this, SLOT(processInputs()));
  connect(fission_example_checkbox_, SIGNAL(clicked()), this, SLOT(processInputs()));
  connect(frame_input_, SIGNAL(editingFinished()), this, SLOT(processInputs()));

  connect(add_button_, SIGNAL(clicked()), this, SLOT(addObstacle()));
  connect(remove_button_, SIGNAL(clicked()), this, SLOT(removeObstacles()));
  connect(reset_button_, SIGNAL(clicked()), this, SLOT(reset()));

  evaluateParams();
}

void ObstaclePublisherPanel::processInputs() {
  verifyInputs();
  setParams();
  evaluateParams();
  notifyParamsUpdate();
}

void ObstaclePublisherPanel::addObstacle() {
  verifyInputs();

  if (r_ > 0.0) {
    p_x_vector_.push_back(x_);
    p_y_vector_.push_back(y_);
    p_r_vector_.push_back(r_);

    p_vx_vector_.push_back(vx_);
    p_vy_vector_.push_back(vy_);

    setParams();
    evaluateParams();
    notifyParamsUpdate();
  }
}

void ObstaclePublisherPanel::removeObstacles() {
  QModelIndexList indexes = obstacles_list_->selectionModel()->selectedIndexes();

  vector<int> index_list;
  for (QModelIndex index : indexes)
    index_list.push_back(index.row());

  sort(index_list.begin(), index_list.end(), greater<int>());

  for (int idx : index_list) {
    p_x_vector_.erase(p_x_vector_.begin() + idx);
    p_y_vector_.erase(p_y_vector_.begin() + idx);
    p_r_vector_.erase(p_r_vector_.begin() + idx);

    p_vx_vector_.erase(p_vx_vector_.begin() + idx);
    p_vy_vector_.erase(p_vy_vector_.begin() + idx);

    delete obstacles_list_items_[idx];
    obstacles_list_items_.erase(obstacles_list_items_.begin() + idx);
  }

  setParams();
  evaluateParams();
  notifyParamsUpdate();
}

void ObstaclePublisherPanel::reset() {
  p_reset_ = true;

  processInputs();

  p_reset_ = false;
}

void ObstaclePublisherPanel::verifyInputs() {
  p_active_ = activate_checkbox_->isChecked();
  p_fusion_example_ = fusion_example_checkbox_->isChecked();
  p_fission_example_ = fission_example_checkbox_->isChecked();

  try { x_ = boost::lexical_cast<double>(x_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { x_ = 0.0; x_input_->setText("0.0"); }

  try { y_ = boost::lexical_cast<double>(y_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { y_ = 0.0; y_input_->setText("0.0"); }

  try { r_ = boost::lexical_cast<double>(r_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { r_ = 0.0; r_input_->setText("0.0"); }

  try { vx_ = boost::lexical_cast<double>(vx_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { vx_ = 0.0; vx_input_->setText("0.0"); }

  try { vy_ = boost::lexical_cast<double>(vy_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { vy_ = 0.0; vy_input_->setText("0.0"); }

  p_frame_id_ = frame_input_->text().toStdString();
}

void ObstaclePublisherPanel::setParams() {
  nh_local_.setParam("active", p_active_);
  nh_local_.setParam("reset", p_reset_);

  nh_local_.setParam("fusion_example", p_fusion_example_);
  nh_local_.setParam("fission_example", p_fission_example_);

  nh_local_.setParam("x_vector", p_x_vector_);
  nh_local_.setParam("y_vector", p_y_vector_);
  nh_local_.setParam("r_vector", p_r_vector_);

  nh_local_.setParam("vx_vector", p_vx_vector_);
  nh_local_.setParam("vy_vector", p_vy_vector_);

  nh_local_.setParam("frame_id", p_frame_id_);
}

void ObstaclePublisherPanel::getParams() {
  p_active_ = nh_local_.param("active", false);
  p_reset_ = nh_local_.param("reset", false);

  p_fusion_example_ = nh_local_.param("fusion_example", false);
  p_fission_example_ = nh_local_.param("fission_example", false);

  nh_local_.getParam("x_vector", p_x_vector_);
  nh_local_.getParam("y_vector", p_y_vector_);
  nh_local_.getParam("r_vector", p_r_vector_);

  nh_local_.getParam("vx_vector", p_vx_vector_);
  nh_local_.getParam("vy_vector", p_vy_vector_);

  p_frame_id_ = nh_local_.param("frame_id", std::string(""));
}

void ObstaclePublisherPanel::evaluateParams() {
  activate_checkbox_->setChecked(p_active_);
  fusion_example_checkbox_->setEnabled(p_active_);
  fission_example_checkbox_->setEnabled(p_active_);

  fusion_example_checkbox_->setChecked(p_fusion_example_);
  fission_example_checkbox_->setChecked(p_fission_example_);

  frame_input_->setEnabled(p_active_);
  frame_input_->setText(QString::fromStdString(p_frame_id_));

  add_button_->setEnabled(p_active_);
  remove_button_->setEnabled(p_active_);
  reset_button_->setEnabled(p_active_);

  x_input_->setEnabled(p_active_);
  y_input_->setEnabled(p_active_);
  r_input_->setEnabled(p_active_);

  vx_input_->setEnabled(p_active_);
  vy_input_->setEnabled(p_active_);

  obstacles_list_->setEnabled(p_active_);

  if (p_x_vector_.size() < p_y_vector_.size() || p_x_vector_.size() < p_r_vector_.size() ||
      p_x_vector_.size() < p_vx_vector_.size() || p_x_vector_.size() < p_vy_vector_.size())
    return;

  for (QListWidgetItem* item : obstacles_list_items_)
    delete item;

  obstacles_list_items_.clear();
  obstacles_list_->clear();

  for (int idx = 0; idx < p_x_vector_.size(); ++idx) {
    QListWidgetItem* item = new QListWidgetItem;
    item->setText(QString::number(idx + 1) + ". " +
                  "[x: " + QString::number(p_x_vector_[idx]) + "m] " +
                  "[y: " + QString::number(p_y_vector_[idx]) + "m] " +
                  "[r: " + QString::number(p_r_vector_[idx]) + "m] " +
                  "[vx: " + QString::number(p_vx_vector_[idx]) + "m/s] " +
                  "[vy: " + QString::number(p_vy_vector_[idx]) + "m/s]");
    obstacles_list_items_.push_back(item);
    obstacles_list_->insertItem(idx, item);
  }
}

void ObstaclePublisherPanel::notifyParamsUpdate() {
  std_srvs::Empty empty;
  if (!params_cli_.call(empty)) {
    p_active_ = false;
    setParams();
    evaluateParams();
  }
}

void ObstaclePublisherPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
}

void ObstaclePublisherPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(obstacle_detector::ObstaclePublisherPanel, rviz::Panel)
