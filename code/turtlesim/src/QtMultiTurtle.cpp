/*
 * Copyright (c) 2009, Willow Garage, Inc.
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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

// Copyright 2024 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file QtMultiTurtle.cpp
*/

#include <docs_turtlesim/QtMultiTurtle.hpp>

#define BG_R 0x45
#define BG_G 0x56
#define BG_B 0xff


MultiTurtleApp::MultiTurtleApp(int& argc, char** argv)
  : QFrame(0, Qt::WindowFlags())
  , path_image_(500, 500, QImage::Format_ARGB32)
  , path_painter_(&path_image_)
{
  setFixedSize(500, 500);
  setWindowTitle("TurtleSim");

  update_timer_ = new QTimer(this);
  update_timer_->setInterval(16);
  update_timer_->start();

  connect(update_timer_, SIGNAL(timeout()), this, SLOT(onUpdate()));

  // make all pixels fully transparent
  path_image_.fill(qRgba(255, 255, 255, 0));
  update();

  // Initialize ROS
  rclcpp::init(argc, argv);
}

MultiTurtleApp::~MultiTurtleApp()
{
  // Shutdown ROS
  rclcpp::shutdown();
}

void MultiTurtleApp::spawnTurtle(const std::string& name, float x, float y, float angle)
{
  QString images_path = (ament_index_cpp::get_package_share_directory("turtlesim") + "/images/").c_str();
  QImage img;
  img.load(images_path + "ardent.png");

  float meter = img.height();
  width_in_meters_ = (width() - 1) / meter;
  height_in_meters_ = (height() - 1) / meter;

  // turtles_.size()+1 is the turtle_id, starting from 1
  std::shared_ptr<Turtle> turtle_node = std::make_shared<Turtle>(turtles_.size()+1, img, QPointF(x, y), angle);
  turtles_.push_back(turtle_node);

  update();
  show();
}

void MultiTurtleApp::onUpdate()
{
  if (!rclcpp::ok())
  {
    close();
    return;
  }

  for (auto& turtle : turtles_)
  {
    rclcpp::spin_some(turtle);
  }

  bool modified = false;
  for (std::shared_ptr<Turtle> turtle : turtles_)
  {
    modified |= turtle->update(0.001 * update_timer_->interval(), path_painter_, path_image_, width_in_meters_, height_in_meters_);
  }
  if (modified)
  {
    update();
  }
}

void MultiTurtleApp::paintEvent(QPaintEvent*)
{
  QPainter painter(this);

  QRgb background_color = qRgb(BG_R, BG_G, BG_B);
  painter.fillRect(0, 0, width(), height(), background_color);

  painter.drawImage(QPoint(0, 0), path_image_);

  for (std::shared_ptr<Turtle> turtle : turtles_)
  {
    turtle->paint(painter);
  }
}

int main(int argc, char** argv)
{
  // Create a new QApplication instance
  QApplication app(argc, argv);

  // Create a new instance of the MultiTurtleApp
  MultiTurtleApp multi_turtles(argc, argv);

  // Spawn the turtles
  multi_turtles.spawnTurtle("turtle1", 2.0, 2.0, 0);
  multi_turtles.spawnTurtle("turtle3", 5.5, 5.5, 0);
  multi_turtles.spawnTurtle("turtle2", 2.0, 8.0, 0);

  // Execute the application
  QApplication::exec();

  return 0;
}
