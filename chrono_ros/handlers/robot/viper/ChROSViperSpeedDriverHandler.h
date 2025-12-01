// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
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

#ifndef CH_VIPER_SPEED_DRIVER_HANDLER_H
#define CH_VIPER_SPEED_DRIVER_HANDLER_H

#include "chrono_ros/ChROSHandler.h"
#include "chrono_models/robot/viper/Viper.h"
#include "rclcpp/subscription.hpp"
#include "std_msgs/msg/float32.hpp"

#include <mutex>

namespace chrono {
namespace ros {

/// @addtogroup ros_robot_handlers
/// @{

/// This handler interfaces a ViperSpeedDriver to ROS 2.
/// It subscribes to a Float32 message representing the desired wheel speed.
class ChROSViperSpeedDriverHandler : public ChROSHandler {
  public:
    /// Constructor. Takes a ViperSpeedDriver instance.
    ChROSViperSpeedDriverHandler(double update_rate,
                                 std::shared_ptr<chrono::viper::ViperSpeedDriver> driver,
                                 const std::string& topic_name);

    /// Initializes the handler.
    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

  protected:
    /// Updates the driver with stored speed data.
    virtual void Tick(double time) override;

  private:
    /// Callback function for speed command subscription.
    void Callback(const std_msgs::msg::Float32& msg);

  private:
    std::shared_ptr<chrono::viper::ViperSpeedDriver> m_driver;  ///< Handle to the driver

    const std::string m_topic_name;                 ///< Topic name for speed commands
    double m_speed_command;                         ///< Latest received speed command
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr m_subscription; ///< ROS 2 subscription for speed input

    std::mutex m_mutex;
};

/// @} ros_robot_handlers

}  // namespace ros
}  // namespace chrono

#endif  // CH_VIPER_SPEED_DRIVER_HANDLER_H
