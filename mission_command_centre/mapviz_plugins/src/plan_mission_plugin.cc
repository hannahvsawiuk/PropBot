// *****************************************************************************
//
// Copyright (c) 2014, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#include <mapviz_plugins/plan_mission_plugin.h>

// C++ standard libraries
#include <cstdio>
#include <vector>

// QT libraries
#include <QDateTime>
#include <QDialog>
#include <QGLWidget>
#include <QMouseEvent>
#include <QPainter>
#include <QPalette>
#include <QStaticText>

#include <opencv2/core/core.hpp>

// ROS libraries
#include <ros/master.h>

#include <swri_route_util/util.h>
#include <swri_transform_util/frames.h>
#include <marti_nav_msgs/PlanRoute.h>


// msg headers
#include <mapviz_plugins/Mission.h>
#include <mapviz_plugins/MissionCommand.h>
#include <mapviz_plugins/MissionCommandCode.h>

// Declare plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mapviz_plugins::PlanMissionPlugin, mapviz::MapvizPlugin)

namespace mnm = marti_nav_msgs;
namespace sru = swri_route_util;
namespace stu = swri_transform_util;

using namespace mapviz_plugins;

/**
 * PlanMissionPlugin constructor
 *
 */

PlanMissionPlugin::PlanMissionPlugin()
    : config_widget_(new QWidget()),
      map_canvas_(NULL),
      route_failed_(false),
      selected_point_(-1),
      is_mouse_down_(false),
      max_ms_(Q_INT64_C(500)),
      max_distance_(2.0) {
  ui_.setupUi(config_widget_);

  ui_.color->setColor(Qt::green);
  // Set background white
  QPalette p(config_widget_->palette());
  p.setColor(QPalette::Background, Qt::white);
  config_widget_->setPalette(p);
  // Set status text red
  QPalette p3(ui_.status->palette());
  p3.setColor(QPalette::Text, Qt::red);
  ui_.status->setPalette(p3);

  // Connect UI button with their respective actions
  QObject::connect(ui_.clear, SIGNAL(clicked()), this, SLOT(Clear()));


  QObject::connect(ui_.upload_mission, SIGNAL(clicked()), this,
                   SLOT(UploadMission()));

  QObject::connect(ui_.start_mission, SIGNAL(clicked()), this,
                   SLOT(StartMissionCommand()));
  QObject::connect(ui_.pause_mission, SIGNAL(clicked()), this,
                   SLOT(PauseMissionCommand()));
  QObject::connect(ui_.resume_mission, SIGNAL(clicked()), this,
                   SLOT(ResumeMissionCommand()));
  QObject::connect(ui_.end_mission, SIGNAL(clicked()), this,
                   SLOT(EndMissionCommand()));
}


/**
 * PlanMissionPlugin destructor
 *
 */

PlanMissionPlugin::~PlanMissionPlugin() {
  if (map_canvas_) {
    map_canvas_->removeEventFilter(this);
  }
}


/**
 * StartMissionCommand function
 *
 * This function publishes a start mission command to the mission_command_ ROS topic.
 *
 */

void PlanMissionPlugin::StartMissionCommand(){
  uint8_t mission_command_code = mapviz_plugins::MissionCommandCode::MISSION_START;
  PrintInfo("Starting mission");
  if(SendMissionCommand( mission_command_code ))
  {
    PrintInfo("Mission started");
  }
  else
  {
    PrintError("Failed to start mission");
  }
  
}


/**
 * PauseMissionCommand function
 *
 * This function publishes a pause mission command to the mission_command_ ROS topic.
 *
 */

void PlanMissionPlugin::PauseMissionCommand(){
  uint8_t mission_command_code = mapviz_plugins::MissionCommandCode::MISSION_PAUSE;
  PrintInfo("Pausing mission");
  if(SendMissionCommand( mission_command_code ))
  {
    PrintInfo("Mission paused");
  }
  else
  {
    PrintError("Failed to pause mission");
  }
}



/**
 * ResumeMissionCommand function
 *
 * This function publishes a start mission command to the mission_command_ ROS topic.
 *
 */

void PlanMissionPlugin::ResumeMissionCommand(){
  uint8_t mission_command_code = mapviz_plugins::MissionCommandCode::MISSION_RESUME;
  PrintInfo("Resuming mission");
  if(SendMissionCommand( mission_command_code ))
  {
    PrintInfo("Mission resumed");
  }
  else
  {
    PrintError("Failed to resume mission");
  }
}

/**
 * EndMissionCommand function
 *
 * This function publishes a start mission command to the mission_command_ ROS topic.
 *
 */

void PlanMissionPlugin::EndMissionCommand(){
  uint8_t mission_command_code = mapviz_plugins::MissionCommandCode::MISSION_END;
  PrintInfo("Ending mission");
  if(SendMissionCommand( mission_command_code ))
  {
    PrintInfo("Mission ended");
  }
  else
  {
    PrintError("Failed to end mission");
  }
}



bool PlanMissionPlugin::SendMissionCommand( uint8_t command_code ){
  mapviz_plugins::MissionCommand mission_command;
  mission_command.header.stamp = ros::Time::now();
  mission_command.command = command_code;
  mission_command_topic_ = ui_.mission_command_topic->text().toStdString();
  if (!mission_command_topic_.empty()) {
    mission_command_pub_.shutdown();
    mission_command_pub_ = node_.advertise<mapviz_plugins::MissionCommand>(mission_command_topic_, 1, true);
    mission_command_pub_.publish(mission_command);
    return true;
  } else {
    PrintError("Mission Topic is empty");
    return false;
  }

}


void PlanMissionPlugin::UploadMission() {

  // return if a single waypoint has not been set 
  if (gps_waypoints_.size() < 1) {
    return;
  }

  mapviz_plugins::Mission mission;

  // configure mission ROS message
  mission.header.frame_id = stu::_wgs84_frame;
  mission.header.stamp = ros::Time::now();
  mission.waypoints = gps_waypoints_;

  // publish mission ROS message to user set topic
  mission_topic_ = ui_.mission_topic->text().toStdString();
  if (!mission_topic_.empty()) {
    mission_pub_.shutdown();
    mission_pub_ = node_.advertise<mapviz_plugins::Mission>(mission_topic_, 1, true);
    mission_pub_.publish(mission);
    PrintInfo("Uploading mission");
  } else {
    PrintError("Mission Topic is empty");
  }
}

/**
 * UploadMission function
 *
 * This function publishes the mission waypoints to a ROS topic.
 *
 */


/**
 * Clear function
 *
 * This function clears mission waypoints from class variable and Mapviz GUI
 *
 */

void PlanMissionPlugin::Clear() {
  gps_waypoints_.clear();
  route_preview_ = sru::RoutePtr();
}


/**
 * PrintError function
 *
 * Prints to plugin status and ROS log level error message 
 *
 */

void PlanMissionPlugin::PrintError(const std::string& message) {
  PrintErrorHelper(ui_.status, message, 1.0);
}


/**
 * PrintInfo function
 *
 * Prints to plugin status and ROS log level info message
 *
 */

void PlanMissionPlugin::PrintInfo(const std::string& message) {
  PrintInfoHelper(ui_.status, message, 1.0);
}


/**
 * PrintWarning function
 *
 * Prints to plugin status and ROS log level warning message
 *
 */

void PlanMissionPlugin::PrintWarning(const std::string& message) {
  PrintWarningHelper(ui_.status, message, 1.0);
}

QWidget* PlanMissionPlugin::GetConfigWidget(QWidget* parent) {
  config_widget_->setParent(parent);

  return config_widget_;
}

bool PlanMissionPlugin::Initialize(QGLWidget* canvas) {
  map_canvas_ = static_cast<mapviz::MapCanvas*>(canvas);
  map_canvas_->installEventFilter(this);

  initialized_ = true;
  return true;
}

bool PlanMissionPlugin::eventFilter(QObject* object, QEvent* event) {
  switch (event->type()) {
    case QEvent::MouseButtonPress:
      return handleMousePress(static_cast<QMouseEvent*>(event));
    case QEvent::MouseButtonRelease:
      return handleMouseRelease(static_cast<QMouseEvent*>(event));
    case QEvent::MouseMove:
      return handleMouseMove(static_cast<QMouseEvent*>(event));
    default:
      return false;
  }
}

bool PlanMissionPlugin::handleMousePress(QMouseEvent* event) {
  selected_point_ = -1;
  int closest_point = 0;
  double closest_distance = std::numeric_limits<double>::max();

#if QT_VERSION >= 0x050000
  QPointF point = event->localPos();
#else
  QPointF point = event->posF();
#endif
  stu::Transform transform;
  if (tf_manager_->GetTransform(target_frame_, stu::_wgs84_frame, transform)) {
    for (size_t i = 0; i < gps_waypoints_.size(); i++) {
      tf::Vector3 waypoint(gps_waypoints_[i].position.x, gps_waypoints_[i].position.y,
                           0.0);
      waypoint = transform * waypoint;

      QPointF transformed = map_canvas_->FixedFrameToMapGlCoord(
          QPointF(waypoint.x(), waypoint.y()));

      double distance = QLineF(transformed, point).length();

      if (distance < closest_distance) {
        closest_distance = distance;
        closest_point = static_cast<int>(i);
      }
    }
  }

  if (event->button() == Qt::LeftButton) {
    if (closest_distance < 15) {
      selected_point_ = closest_point;
      return true;
    } else {
      is_mouse_down_ = true;
#if QT_VERSION >= 0x050000
      mouse_down_pos_ = event->localPos();
#else
      mouse_down_pos_ = event->posF();
#endif
      mouse_down_time_ = QDateTime::currentMSecsSinceEpoch();
      return false;
    }
  } else if (event->button() == Qt::RightButton) {
    if (closest_distance < 15) {
      gps_waypoints_.erase(gps_waypoints_.begin() + closest_point);
      return true;
    }
  }

  return false;
}

bool PlanMissionPlugin::handleMouseRelease(QMouseEvent* event) {
#if QT_VERSION >= 0x050000
  QPointF point = event->localPos();
#else
  QPointF point = event->posF();
#endif
  if (selected_point_ >= 0 &&
      static_cast<size_t>(selected_point_) < gps_waypoints_.size()) {
    stu::Transform transform;
    if (tf_manager_->GetTransform(stu::_wgs84_frame, target_frame_,
                                  transform)) {
      QPointF transformed = map_canvas_->MapGlCoordToFixedFrame(point);
      tf::Vector3 position(transformed.x(), transformed.y(), 0.0);
      position = transform * position;
      gps_waypoints_[selected_point_].position.x = position.x();
      gps_waypoints_[selected_point_].position.y = position.y();
    }

    selected_point_ = -1;
    return true;
  } else if (is_mouse_down_) {
    qreal distance = QLineF(mouse_down_pos_, point).length();
    qint64 msecsDiff = QDateTime::currentMSecsSinceEpoch() - mouse_down_time_;

    // Only fire the event if the mouse has moved less than the maximum distance
    // and was held for shorter than the maximum time..  This prevents click
    // events from being fired if the user is dragging the mouse across the map
    // or just holding the cursor in place.
    if (msecsDiff < max_ms_ && distance <= max_distance_) {
      QPointF transformed = map_canvas_->MapGlCoordToFixedFrame(point);

      stu::Transform transform;
      tf::Vector3 position(transformed.x(), transformed.y(), 0.0);
      if (tf_manager_->GetTransform(stu::_wgs84_frame, target_frame_,
                                    transform)) {
        position = transform * position;

        geometry_msgs::Pose pose;
        pose.position.x = position.x();
        pose.position.y = position.y();
        gps_waypoints_.push_back(pose);
      }
    }
  }
  is_mouse_down_ = false;

  return false;
}

bool PlanMissionPlugin::handleMouseMove(QMouseEvent* event) {
  if (selected_point_ >= 0 &&
      static_cast<size_t>(selected_point_) < gps_waypoints_.size()) {
#if QT_VERSION >= 0x050000
    QPointF point = event->localPos();
#else
    QPointF point = event->posF();
#endif
    stu::Transform transform;
    if (tf_manager_->GetTransform(stu::_wgs84_frame, target_frame_,
                                  transform)) {
      QPointF transformed = map_canvas_->MapGlCoordToFixedFrame(point);
      tf::Vector3 position(transformed.x(), transformed.y(), 0.0);
      position = transform * position;
      gps_waypoints_[selected_point_].position.y = position.y();
      gps_waypoints_[selected_point_].position.x = position.x();
    }

    return true;
  }
  return false;
}

void PlanMissionPlugin::Draw(double x, double y, double scale) {
  stu::Transform transform;
  if (tf_manager_->GetTransform(target_frame_, stu::_wgs84_frame, transform)) {
    if (!route_failed_) {
      if (route_preview_) {
        sru::Route route = *route_preview_;
        sru::transform(route, transform, target_frame_);

        glLineWidth(2);
        const QColor color = ui_.color->color();
        glColor4d(color.redF(), color.greenF(), color.blue(), 1.0);
        glBegin(GL_LINE_STRIP);

        for (size_t i = 0; i < route.points.size(); i++) {
          glVertex2d(route.points[i].position().x(),
                     route.points[i].position().y());
        }
        glEnd();
      }
    }
    // Draw waypoints

    glPointSize(20);
    glColor4f(0.0, 1.0, 1.0, 1.0);
    glBegin(GL_POINTS);

    for (size_t i = 0; i < gps_waypoints_.size(); i++) {
      tf::Vector3 point(gps_waypoints_[i].position.x, gps_waypoints_[i].position.y, 0);
      point = transform * point;
      glVertex2d(point.x(), point.y());
    }
    glEnd();
  } else {
    PrintError("Waypoint transform cannot be retrieved");
  }
}

void PlanMissionPlugin::Paint(QPainter* painter, double x, double y,
                            double scale) {
  painter->save();
  painter->resetTransform();

  QPen pen(QBrush(QColor(Qt::darkCyan).darker()), 1);
  painter->setPen(pen);
  painter->setFont(QFont("DejaVu Sans Mono", 7));

  stu::Transform transform;
  if (tf_manager_->GetTransform(target_frame_, stu::_wgs84_frame, transform)) {
    for (size_t i = 0; i < gps_waypoints_.size(); i++) {
      tf::Vector3 point(gps_waypoints_[i].position.x, gps_waypoints_[i].position.y, 0);
      point = transform * point;
      QPointF gl_point =
          map_canvas_->FixedFrameToMapGlCoord(QPointF(point.x(), point.y()));
      QPointF corner(gl_point.x() - 20, gl_point.y() - 20);
      QRectF rect(corner, QSizeF(40, 40));
      painter->drawText(
          rect, Qt::AlignHCenter | Qt::AlignVCenter,
          QString::fromStdString(boost::lexical_cast<std::string>(i + 1)));
    }
  }

  painter->restore();
}

void PlanMissionPlugin::LoadConfig(const YAML::Node& node,
                                 const std::string& path) {
  if (node["route_topic"]) {
    std::string route_topic;
    node["route_topic"] >> route_topic;
    ui_.route_topic->setText(route_topic.c_str());
  }
  if (node["color"]) {
    std::string color;
    node["color"] >> color;
    ui_.color->setColor(QColor(color.c_str()));fabsf64;
  }

  if (node["mission_topic"]) {
    std::string mission_topic;
    node["mission_topic"] >> mission_topic;
    ui_.mission_topic->setText(mission_topic.c_str());
  }
  if (node["start_from_vehicle"]) {
    bool start_from_vehicle;
    node["start_from_vehicle"] >> start_from_vehicle;
    ui_.start_from_vehicle->setChecked(start_from_vehicle);
  }

  if (node["mission_command_topic"]) {
    std::string mission_command_topic;
    node["mission_command_topic"] >> mission_command_topic;
    ui_.mission_command_topic->setText(mission_command_topic.c_str());
  }
}

void PlanMissionPlugin::SaveConfig(YAML::Emitter& emitter,
                                 const std::string& path) {
  emitter << YAML::Key << "route_topic" << YAML::Value << route_topic_;

  std::string color = ui_.color->color().name().toStdString();
  emitter << YAML::Key << "color" << YAML::Value << color;

  std::string mission_topic = ui_.mission_topic->text().toStdString();
  emitter << YAML::Key << "mission_topic" << YAML::Value << mission_topic_;

  bool start_from_vehicle = ui_.start_from_vehicle->isChecked();
  emitter << YAML::Key << "start_from_vehicle" << YAML::Value
          << start_from_vehicle;

  std::string mission_command_topic = ui_.mission_command_topic->text().toStdString();
  emitter << YAML::Key << "mission_command_topic" << YAML::Value << mission_command_topic_;
}
