#include "sensor_msgs/TimeReference.h"
#include "bluefox2/single_node.h"
#include "bluefox2/bluefox2_ros.h"
#include <ros/ros.h>
#include <iostream>

namespace bluefox2 {

bool flag = 0;
bool firstCall = 1;
ros::Time triggerTime;
uint32_t triggerCounter;
uint32_t nextTriggerCounter;

SingleNode::SingleNode(const ros::NodeHandle& pnh)
    : CameraNodeBase(pnh),
      bluefox2_ros_(boost::make_shared<Bluefox2Ros>(pnh)),
      nh(pnh)
      {
     
        subTimeRef = nh.subscribe("/imu/trigger_time", 1000, &bluefox2::SingleNode::callback, this);
      
      }

void SingleNode::Acquire() {
  while (is_acquire() && ros::ok()) {
    bluefox2_ros_->RequestSingle();
    
    while (flag==0) {
      //ROS_WARN_STREAM("Esperando ");
      ros::Duration(0.001).sleep();
    }
    flag = 0;
    
    if (triggerCounter == nextTriggerCounter) { // a new video frame was captured, check if we need to skip it if one trigger packet was lost
        ROS_INFO_STREAM("Frame Published");
        const auto expose_us = bluefox2_ros_->camera().GetExposeUs();
        const auto expose_duration = ros::Duration(expose_us * 1e-6 / 2);
        const auto timeStamp = triggerTime + expose_duration;
        bluefox2_ros_->PublishCamera(timeStamp);
    } 
    else { 
        ROS_WARN("trigger not in sync (seq expected %10u, got %10u)!", nextTriggerCounter, triggerCounter);     
    } 
    nextTriggerCounter++;
    Sleep();
  }
}

void SingleNode::AcquireOnce() {
  if (is_acquire() && ros::ok()) {

    bluefox2_ros_->RequestSingle();
    const auto expose_us = bluefox2_ros_->camera().GetExposeUs();
    const auto expose_duration = ros::Duration(expose_us * 1e-6 / 2);
    const auto time = ros::Time::now() + expose_duration;
    bluefox2_ros_->PublishCamera(time);
  }
}

void SingleNode::Setup(Bluefox2DynConfig& config) {
  bluefox2_ros_->set_fps(config.fps);
  bluefox2_ros_->camera().Configure(config);
}

void SingleNode::callback(const sensor_msgs::TimeReference::ConstPtr &time_ref) {
  if (firstCall == 1){
    nextTriggerCounter=time_ref->header.seq;
    firstCall = 0;
    ROS_INFO_STREAM("First Callback");
  }
  triggerCounter = time_ref->header.seq; 	
  triggerTime = time_ref->header.stamp;
  ROS_INFO_STREAM("New Frame: " << triggerCounter);
  flag = 1;
}

}  // namepace bluefox2



