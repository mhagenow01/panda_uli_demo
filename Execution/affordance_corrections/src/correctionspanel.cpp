// File name:  correctionspanel.cpp (based on drone panel)
// Description: implementation related to custom rviz panel for drone viewpoints
// Author: Mike Hagenow
// Date: 3/31/22
// Using information found in the ROS
// custom plugins tutorial: https://github.com/ros-visualization/visualization_tutorials/blob/groovy-devel/rviz_plugin_tutorials/src/teleop_panel.cpp

#include <stdio.h>
#include "correctionspanel.h"

#include <QWidget>
#include <QPainter>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFont>
#include <QSizePolicy>
#include <QString>
#include <QScreen>
#include <QGuiApplication>

#include <ros/console.h>
#include <tf2_ros/transform_broadcaster.h>

#include <rviz/visualization_manager.h>
#include <rviz/view_controller.h>
#include <rviz/render_panel.h>
#include <rviz/view_manager.h>
#include <OGRE/OgreCamera.h>
#include <OgreQuaternion.h>
#include <OgreVector3.h>


namespace corrections_panel{

    CorrectionsPanel::CorrectionsPanel(QWidget* parent): rviz::Panel(parent){

        int screenWidth = QGuiApplication::primaryScreen()->geometry().size().width();
        int screenRatio = screenWidth/1920;

        cf = 0; //control frame is originally global robot
        mapping = false;

        // Initialize publishers
        cam_pos_pub = n.advertise<geometry_msgs::Point>("rviz_camera_p", 1);
        quat_pub = n.advertise<geometry_msgs::Quaternion>("rviz_camera_q", 1);
        rviz_pub = n.advertise<std_msgs::String>("getObjPose", 1);

        static tf2_ros::TransformBroadcaster br;

        QWidget *cfBox = new QWidget;
        QHBoxLayout* cfLayout = new QHBoxLayout(cfBox);
        cfBox->setStyleSheet("background-color: #dae3e3; border-radius: 10pt; border-color: #b6b8b8");
        cfBox->setFixedWidth(500*screenRatio);
        QPushButton* toggleControlbutton = new QPushButton("Run Behavior");
        toggleControlbutton->setStyleSheet("background-color: #B6D5E7; border-style: solid; border-width: 2pt; border-radius: 10pt; border-color: #B6D5E7; font: bold 18pt; min-width: 10em; padding: 6pt;");
        cfLayout->addWidget(toggleControlbutton);


        QHBoxLayout* hlayout = new QHBoxLayout;
        hlayout->addWidget(cfBox);
        setLayout(hlayout);

        // Turn on and off mapping
        connect(toggleControlbutton, &QPushButton::clicked, [this,toggleMappingbutton](){
           if(!mapping){
            s_out.data = "on";
            toggleMappingbutton->setText("End Mapping");
            toggleMappingbutton->setStyleSheet("background-color: #FF968A; border-style: solid; border-width: 2pt; border-radius: 10pt; border-color: #B6D5E7; font: bold 18pt; min-width: 10em; padding: 6pt;");
            mapping = true;
           }
           else{
               s_out.data = "off";
               toggleMappingbutton->setText("Start Mapping");
               toggleMappingbutton->setStyleSheet("background-color: #B6D5E7; border-style: solid; border-width: 2pt; border-radius: 10pt; border-color: #B6D5E7; font: bold 18pt; min-width: 10em; padding: 6pt;");
               mapping = false;
           }
           rviz_pub.publish(s_out);
        });

        // Go to current alternate view
        connect(altViewbutton, &QPushButton::clicked, [this,altViewbutton](){
            // altViewbutton->setEnabled(false);
            s_out.data = "go";
            altview_pub.publish(s_out);
        });

        
        // Timer used to publish the camera orientation from RVIZ for camera-centric controls
        QTimer* output_timer = new QTimer( this );  
        connect(output_timer, &QTimer::timeout, [this](){
            rviz::ViewManager* viewm = vis_manager_->getViewManager();
            rviz::ViewController* vc = viewm->getCurrent();
            Ogre::Camera* camera = vc->getCamera();
            const Ogre::Quaternion quat = camera->getOrientation();
            const Ogre::Vector3 cam_pos = camera->getPosition();
            
            geometry_msgs::TransformStamped transformStamped;
            transformStamped.header.stamp = ros::Time::now();
            transformStamped.header.frame_id = "panda_link0";
            transformStamped.child_frame_id = "rvizcamera";

            transformStamped.transform.translation.x = cam_pos.x;
            transformStamped.transform.translation.y = cam_pos.y;
            transformStamped.transform.translation.z = cam_pos.z;
            transformStamped.transform.rotation.x = quat.x;
            transformStamped.transform.rotation.y = quat.y;
            transformStamped.transform.rotation.z = quat.z;
            transformStamped.transform.rotation.w = quat.w;

            br.sendTransform(transformStamped);
        }); 
        output_timer->start(100);

    }


} // end namespace

// Make pluginlib aware of the class
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(corrections_panel::CorrectionsPanel,rviz::Panel)
