#ifndef CH_VIPER_WAYPOINT_PATH_HANDLER_H
#define CH_VIPER_WAYPOINT_PATH_HANDLER_H

#include "chrono_ros/ChROSHandler.h"
#include "chrono_ros/handlers/ChROSHandlerUtilities.h"

#include "chrono_models/robot/viper/Viper.h"

#include <nav_msgs/msg/path.hpp>

namespace chrono {
namespace ros {

class ChROSViperWaypointPathHandler : public ChROSHandler {
  public:
    ChROSViperWaypointPathHandler(double update_rate,
                                  std::shared_ptr<chrono::viper::ViperWaypointFollower> driver,
                                  const std::string& topic_name,
                                  const std::string& frame_id = "map");

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;
    virtual void Tick(double time) override;

  private:
    void PublishPath(double time);

    std::shared_ptr<chrono::viper::ViperWaypointFollower> m_driver;
    std::string m_topic_name;
    std::string m_frame_id;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_path_publisher;
    nav_msgs::msg::Path m_path_msg;
};

}  // namespace ros
}  // namespace chrono

#endif
