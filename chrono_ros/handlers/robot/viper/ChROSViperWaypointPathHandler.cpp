#include "chrono_ros/handlers/robot/viper/ChROSViperWaypointPathHandler.h"

#include "chrono_ros/handlers/ChROSHandlerUtilities.h"

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace chrono {
namespace ros {

ChROSViperWaypointPathHandler::ChROSViperWaypointPathHandler(
    double update_rate,
    std::shared_ptr<chrono::viper::ViperWaypointFollower> driver,
    const std::string& topic_name,
    const std::string& frame_id)
    : ChROSHandler(update_rate), m_driver(driver), m_topic_name(topic_name), m_frame_id(frame_id) {}

bool ChROSViperWaypointPathHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    auto node = interface->GetNode();

    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, m_topic_name)) {
        return false;
    }

    m_path_publisher = node->create_publisher<nav_msgs::msg::Path>(m_topic_name, 1);
    m_path_msg.header.frame_id = m_frame_id;

    return true;
}

void ChROSViperWaypointPathHandler::Tick(double time) {
    PublishPath(time);
}

void ChROSViperWaypointPathHandler::PublishPath(double time) {
    if (!m_path_publisher || !m_driver) {
        return;
    }

    auto path_points = m_driver->GetPathPoints();

    m_path_msg.header.stamp = ChROSHandlerUtilities::GetROSTimestamp(time);
    m_path_msg.poses.clear();
    m_path_msg.poses.reserve(path_points.size());

    for (const auto& point : path_points) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = m_path_msg.header;

        pose.pose.position.x = point.x();
        pose.pose.position.y = point.y();
        pose.pose.position.z = point.z();

        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;

        m_path_msg.poses.push_back(pose);
    }

    m_path_publisher->publish(m_path_msg);
}

}  // namespace ros
}  // namespace chrono
