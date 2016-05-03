// Copyright 2016 AUV-IITK

// header guard
#ifndef HARDWARE_CAMERA_RESIZED_SUBSCRIBER_H
#define HARDWARE_CAMERA_RESIZED_SUBSCRIBER_H
#endif  // HARDWARE_CAMERA_RESIZED_SUBSCRIBER_H

#include <image_transport/simple_subscriber_plugin.h>
#include <hardware_camera/ResizedImage.h>
#include <string>

class ResizedSubscriber : public image_transport::SimpleSubscriberPlugin<hardware_camera::ResizedImage>
{
public:
  virtual ~ResizedSubscriber()
  {
  }

  virtual std::string getTransportName() const
  {
    return "resized";
  }

protected:
  virtual void internalCallback(const typename hardware_camera::ResizedImage::ConstPtr &message,
                                const Callback &user_cb);
};
