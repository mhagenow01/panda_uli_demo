// File name:  correctionspanel.h
// Description: header related to custom rviz panel for visual information and publishing
// Author: Mike Hagenow
// Date: 3/31/22
// Using information found in the ROS
// custom plugins tutorial: https://github.com/ros-visualization/visualization_tutorials/blob/groovy-devel/rviz_plugin_tutorials/src/teleop_panel.cpp

#ifndef CORRECTIONS_PANEL_H
#define CORRECTIONS_PANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/TransformStamped.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Bool.h"
#include <QLabel>
#include <QPushButton>
#include <QCheckBox>

namespace corrections_panel
{
    // forward declaration to avoid "does not name a type" cyclic dependency
    class DriveWidget;

    class CorrectionsPanel: public rviz::Panel{
        public:
            CorrectionsPanel( QWidget* parent = 0);
            void refitCallback(std_msgs::Bool data);
            void rvizCallback(std_msgs::String data);
        protected:
            DriveWidget* drive_widget_;
        private:
            ros::NodeHandle n;
            ros::Publisher quat_pub;
            ros::Publisher cam_pos_pub;
            ros::Publisher trigger_pub;
            ros::Subscriber refit_sub;
            ros::Subscriber rviz_sub;
            geometry_msgs::Quaternion q_out;
            geometry_msgs::Point pos_out;
            ros::Subscriber battery_sub;
            ros::Publisher cf_pub;
            ros::Publisher obj_pub;
            ros::Publisher altview_pub;
            ros::Publisher exec_pub;
            ros::Publisher pause_pub;
            bool mapping;
            int fragmentid;
            QLabel* bat;
            std_msgs::String s_out;

            QCheckBox* refitting;
            QPushButton* scanButton;
            QPushButton* toggleBehaviorbutton;

            std_msgs::Int8 artic_out;
            ros::Publisher artic_pub;
            
            bool trial_running;
            
    };
} // end of namespace
#endif