// Copyright 2016 AUV-IITK
#include <image_transport/simple_subscriber_plugin.h>
#include <hardware_camera/ResizedImage.h>

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
