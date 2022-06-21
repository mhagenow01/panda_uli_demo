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

        mapping = false;
        fragmentid = 0; // 0 - ready to compute, 1 - ready to execute

        // Initialize publishers
        cam_pos_pub = n.advertise<geometry_msgs::Point>("rviz_camera_p", 1);
        quat_pub = n.advertise<geometry_msgs::Quaternion>("rviz_camera_q", 1);
        trigger_pub = n.advertise<std_msgs::String>("rviz_triggers", 1);
        obj_pub = n.advertise<std_msgs::String>("getObjPose", 1);
        pause_pub = n.advertise<std_msgs::String>("/execution/interrupt", 1);
        exec_pub = n.advertise<std_msgs::String>("executeModel", 1);
        refit_sub = n.subscribe("objRefittingUpdate", 1, &CorrectionsPanel::refitCallback, this);
        rviz_sub = n.subscribe("rviz_triggers", 1, &CorrectionsPanel::rvizCallback, this);

        static tf2_ros::TransformBroadcaster br;

        QWidget *cfBox = new QWidget;
        QVBoxLayout* cfLayout = new QVBoxLayout(cfBox);
        cfBox->setStyleSheet("background-color: #dae3e3; border-radius: 10pt; border-color: #b6b8b8");
        cfBox->setFixedHeight(150*screenRatio);

        scanButton = new QPushButton("Run Scan");
        scanButton->setStyleSheet("background-color: #B6D5E7; border-style: solid; border-width: 2pt; border-radius: 10pt; border-color: #B6D5E7; font: bold 18pt; min-width: 10em; padding: 6pt;");
        cfLayout->addWidget(scanButton);

        QPushButton* deletebutton = new QPushButton("Delete Object");
        deletebutton->setStyleSheet("background-color: #FF968A; border-style: solid; border-width: 2px; border-radius: 10px; border-color: #FF968A; font: bold 22px; min-width: 10em; padding: 6px;");
        cfLayout->addWidget(deletebutton);

        toggleBehaviorbutton = new QPushButton("Compute Trajectory");
        toggleBehaviorbutton->setStyleSheet("background-color: #B6D5E7; border-style: solid; border-width: 2pt; border-radius: 10pt; border-color: #B6D5E7; font: bold 18pt; min-width: 10em; padding: 6pt;");
        cfLayout->addWidget(toggleBehaviorbutton);

        refitting = new QCheckBox("Refitting Enabled");
        refitting->setChecked(true);
        refitting->setStyleSheet("font: bold 18px; min-width: 10em; padding-left:130%; margin-left:50%; margin-right:50%;");
        cfLayout->addWidget(refitting);

        QHBoxLayout* hlayout = new QHBoxLayout;
        hlayout->addWidget(cfBox);
        setLayout(hlayout);

        // Run Scan
        connect(scanButton, &QPushButton::clicked, [this](){
           if(!mapping){
               mapping = true;
               s_out.data = "scan";
               trigger_pub.publish(s_out);
               scanButton->setEnabled(false);
               scanButton->setText("Scanning");
           }  
        });

        // Run Behavior
        connect(toggleBehaviorbutton, &QPushButton::clicked, [this](){
           if(fragmentid==0){ // ready to compute
                s_out.data = "on";
                obj_pub.publish(s_out);
                toggleBehaviorbutton->setEnabled(false);
                toggleBehaviorbutton->setText("Computing");
           }
           else if(fragmentid==1){ //ready to execute
                s_out.data = "execute";
                exec_pub.publish(s_out);
                toggleBehaviorbutton->setEnabled(false);
                toggleBehaviorbutton->setText("Executing");
           }
           else if(fragmentid==2){ //pause
                s_out.data = "pause";
                pause_pub.publish(s_out);
                toggleBehaviorbutton->setEnabled(false);
                toggleBehaviorbutton->setText("Pausing...");
           }
            else if(fragmentid==3){ //resume
                s_out.data = "resume";
                pause_pub.publish(s_out);
                toggleBehaviorbutton->setEnabled(false);
                toggleBehaviorbutton->setText("Resuming...");
           }
        });

        // Delete Object
        connect(deletebutton, &QPushButton::clicked, [this](){
           s_out.data = "deleteactive";
           trigger_pub.publish(s_out);
        });

        // Toggle refitting
        connect(refitting, &QCheckBox::stateChanged, [this](){
           if(refitting->isChecked()){
               s_out.data = "refittingon";
           }
           else{
               s_out.data = "refittingoff";
           }
           
           trigger_pub.publish(s_out);
        });

        
        // Timer used to publish the camera orientation from RVIZ for camera-centric controls
        QTimer* output_timer = new QTimer( this );  
        connect(output_timer, &QTimer::timeout, [this](){
            rviz::ViewManager* viewm = vis_manager_->getViewManager();
            rviz::ViewController* vc = viewm->getCurrent();
            Ogre::Camera* camera = vc->getCamera();
            const Ogre::Quaternion quat = camera->getOrientation();
            const Ogre::Vector3 cam_pos = camera->getPosition();

            // Convert from Ogre to ROS message
            q_out.x = quat.x; q_out.y = quat.y; q_out.z = quat.z; q_out.w = quat.w;
            pos_out.x = cam_pos.x; pos_out.y = cam_pos.y; pos_out.z = cam_pos.z; 
            quat_pub.publish(q_out);
            cam_pos_pub.publish(pos_out);

            ros::spinOnce();
            
            // geometry_msgs::TransformStamped transformStamped;
            // transformStamped.header.stamp = ros::Time::now();
            // transformStamped.header.frame_id = "panda_link0";
            // transformStamped.child_frame_id = "rvizcamera";

            // transformStamped.transform.translation.x = cam_pos.x;
            // transformStamped.transform.translation.y = cam_pos.y;
            // transformStamped.transform.translation.z = cam_pos.z;
            // transformStamped.transform.rotation.x = quat.x;
            // transformStamped.transform.rotation.y = quat.y;
            // transformStamped.transform.rotation.z = quat.z;
            // transformStamped.transform.rotation.w = quat.w;

            // br.sendTransform(transformStamped);
        }); 
        output_timer->start(100);

    }

    void CorrectionsPanel::refitCallback(std_msgs::Bool data){
            // Update the refitting checkbox for the active object
            refitting->setChecked(data.data);
    }

    void CorrectionsPanel::rvizCallback(std_msgs::String data){
            if(data.data=="scanningdone"){
                scanButton->setEnabled(true);
                mapping = false;
                scanButton->setText("Run Scan");
            }
            if(data.data=="computetrajdone"){
                toggleBehaviorbutton->setEnabled(true);
                toggleBehaviorbutton->setText("Execute Trajectory");
                fragmentid = 1; // ready to execute
            }
            if(data.data=="execdone"){
                toggleBehaviorbutton->setEnabled(true);
                toggleBehaviorbutton->setText("Compute Trajectory");
                fragmentid = 0; // ready to compute
            }
            if(data.data=="enablepausebutton"){
                toggleBehaviorbutton->setEnabled(true);
                toggleBehaviorbutton->setText("Pause Execution");
                fragmentid = 2; // ready to compute
            }
            if(data.data=="enableresumebutton"){
                toggleBehaviorbutton->setEnabled(true);
                toggleBehaviorbutton->setText("Resume Execution");
                fragmentid = 3; // ready to compute
            }
    }



} // end namespace

// Make pluginlib aware of the class
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(corrections_panel::CorrectionsPanel,rviz::Panel)
