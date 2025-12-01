// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Keshav Sharan
// =============================================================================
//
// Handler responsible for receiving and updating the DC motor control commands
// for the Viper rover
//
// =============================================================================

#ifndef CH_VIPER_WAYPOINT_FOLLOWER_HANDLER_H
#define CH_VIPER_WAYPOINT_FOLLOWER_HANDLER_H

#include "chrono_ros/ChROSHandler.h"
#include "chrono_models/robot/viper/Viper.h"
#include "rclcpp/subscription.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include <mutex>

namespace chrono {
namespace ros {

/// @addtogroup ros_robot_handlers
/// @{

/// This handler is responsible for interfacing a ViperWaypointFollower driver to ROS.
class ChROSViperWaypointFollowerHandler : public ChROSHandler {
  public:
    /// Constructor. Takes a ViperWaypointFollower driver
    ChROSViperWaypointFollowerHandler(double update_rate,
                                    std::shared_ptr<chrono::viper::ViperWaypointFollower> driver,
                                    const std::string& topic_name);

    /// Initializes the handler.
    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

  protected:
    /// Updates the driver with stored inputs data from Callback
    virtual void Tick(double time) override;

  private:
    /// NOTE: This will only update the local m_inputs variable. The driver will receive
    /// the new commands in the Tick() function.
    void Callback(const geometry_msgs::msg::Vector3& msg);

  private:
    std::shared_ptr<chrono::viper::ViperWaypointFollower> m_driver;  ///< handle to the driver

    const std::string m_topic_name;                         ///< name of the topic to publish to
    double m_target_x;                               ///< Latest received first speed command
    double m_target_y; 
    double m_target_z;                              ///< Latest received second speed command
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr m_subscription; ///< ROS 2 subscription

    std::mutex m_mutex;
};

/// @} ros_robot_handlers

}  // namespace ros
}  // namespace chrono

#endif
