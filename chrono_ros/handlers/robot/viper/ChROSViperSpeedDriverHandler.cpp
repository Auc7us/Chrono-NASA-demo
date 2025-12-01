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

#include "chrono_ros/handlers/robot/viper/ChROSViperSpeedDriverHandler.h"
#include "chrono_ros/handlers/ChROSHandlerUtilities.h"

using std::placeholders::_1;

using namespace chrono::viper;

namespace chrono {
namespace ros {

ChROSViperSpeedDriverHandler::ChROSViperSpeedDriverHandler(double update_rate,
                                                           std::shared_ptr<ViperSpeedDriver> driver,
                                                           const std::string& topic_name)
    : ChROSHandler(update_rate), m_driver(driver), m_topic_name(topic_name), m_speed_command(0.0) {}

bool ChROSViperSpeedDriverHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    auto node = interface->GetNode();

    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, m_topic_name)) {
        return false;
    }

    m_subscription = node->create_subscription<std_msgs::msg::Float32>(
        m_topic_name, 10, std::bind(&ChROSViperSpeedDriverHandler::Callback, this, _1));

    return true;
}

void ChROSViperSpeedDriverHandler::Callback(const std_msgs::msg::Float32& msg) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_speed_command = msg.data;
}

void ChROSViperSpeedDriverHandler::Tick(double time) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_driver->SetSpeed(m_speed_command);
}

}  // namespace ros
}  // namespace chrono
