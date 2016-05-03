#include <image_transport/simple_publisher_plugin.h>
#include <hardware_camera/ResizedImage.h>

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
