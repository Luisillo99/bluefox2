#ifndef BLUEFOX2_SINGLE_NODE_H_
#define BLUEFOX2_SINGLE_NODE_H_

#include "ros/ros.h"
#include "sensor_msgs/TimeReference.h"
#include "bluefox2/Bluefox2DynConfig.h"
#include <camera_base/camera_node_base.h>
#include <iostream>

namespace bluefox2 {

class Bluefox2Ros;

class SingleNode : public camera_base::CameraNodeBase<Bluefox2DynConfig> {
 public:
  explicit SingleNode(const ros::NodeHandle &pnh);

  virtual void Acquire() override;
  virtual void Setup(Bluefox2DynConfig &config) override;

  void AcquireOnce();
  void callback(const sensor_msgs::TimeReference::ConstPtr &time_ref);
 private:
  boost::shared_ptr<Bluefox2Ros> bluefox2_ros_;
  ros::Subscriber subTimeRef;
  ros::NodeHandle nh;
  bool boost_{false};
};

}  // namespace bluefox2

#endif  // BLUEFOX2_SINGLE_NODE_H_
