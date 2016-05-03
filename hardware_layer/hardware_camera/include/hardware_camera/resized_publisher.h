// Copyright 2016 AUV-IITK

// header guard
#ifndef HARDWARE_CAMERA_RESIZED_PUBLISHER_H
#define HARDWARE_CAMERA_RESIZED_PUBLISHER_H
#endif  // HARDWARE_CAMERA_RESIZED_PUBLISHER_H

#include <image_transport/simple_publisher_plugin.h>
#include <hardware_camera/ResizedImage.h>
#include <string>

class ResizedPublisher : public image_transport::SimplePublisherPlugin<hardware_camera::ResizedImage>
{
public:
  virtual std::string getTransportName() const
  {
    return "resized";
  }

protected:
  virtual void publish(const sensor_msgs::Image &message, const PublishFn &publish_fn) const;
};
