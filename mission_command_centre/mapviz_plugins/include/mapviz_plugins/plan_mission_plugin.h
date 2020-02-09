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

#pragma once


#include <string>
#include <vector>

#include <mapviz/mapviz_plugin.h>

#include <QGLWidget>
#include <QObject>
#include <QWidget>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <mapviz/map_canvas.h>
#include <swri_route_util/route.h>

#include <geometry_msgs/Pose.h>
#include <marti_nav_msgs/Route.h>

#include "ui_plan_mission_config.h"

namespace mapviz_plugins
{

/**
 * PlanMissionPlugin
 *
 * This plugin class allows the user to set mission waypoints on the Mapviz GUI and upload them to the robot
 *
 */
  class PlanMissionPlugin : public mapviz::MapvizPlugin
  {
    Q_OBJECT

   public:

    PlanMissionPlugin();
    virtual ~PlanMissionPlugin();

    bool Initialize(QGLWidget* canvas) override;
    void Shutdown() override {}

    void Draw(double x, double y, double scale) override;
    void Paint(QPainter* painter, double x, double y, double scale) override;

    void Transform() override {}

    void LoadConfig(const YAML::Node& node, const std::string& path) override;
    void SaveConfig(YAML::Emitter& emitter, const std::string& path) override;

    QWidget* GetConfigWidget(QWidget* parent) override;

    bool SupportsPainting() override
    {
      return true;
    }

   protected:
    void PrintError(const std::string& message) override;
    void PrintInfo(const std::string& message) override;
    void PrintWarning(const std::string& message) override;
    bool eventFilter(QObject *object, QEvent* event) override;
    bool handleMousePress(QMouseEvent *);
    bool handleMouseRelease(QMouseEvent *);
    bool handleMouseMove(QMouseEvent *);

   protected Q_SLOTS:
    void Clear();
    void UploadMission();

   private:
    Ui::plan_mission_config ui_;
    QWidget* config_widget_;
    mapviz::MapCanvas* map_canvas_;

    std::string route_topic_;
    std::string mission_topic_;

    ros::Publisher mission_pub_;
    ros::Subscriber route_sub_;
    ros::Timer retry_timer_;

    bool route_failed_;
    swri_route_util::RoutePtr route_preview_;

    std::vector<geometry_msgs::Pose> gps_waypoints_;

    int selected_point_;
    bool is_mouse_down_;
    QPointF mouse_down_pos_;
    qint64 mouse_down_time_;

    qint64 max_ms_;
    qreal max_distance_;

  };
}
