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

#include "chrono_ros/handlers/robot/viper/ChROSViperWaypointFollowerHandler.h"
#include "chrono_ros/handlers/ChROSHandlerUtilities.h"

using std::placeholders::_1;
using namespace chrono::viper;

namespace chrono {
namespace ros {

ChROSViperWaypointFollowerHandler::ChROSViperWaypointFollowerHandler(double update_rate,
                                                                 std::shared_ptr<ViperWaypointFollower> driver,
                                                                 const std::string& topic_name)
    : ChROSHandler(update_rate), m_driver(driver), m_topic_name(topic_name) {

        // ChVector3d start_pos = driver->GetVehicle()->GetChassisPos();
        m_target_x = -5;
        m_target_y = 0;
        m_target_z = 0;
    }

bool ChROSViperWaypointFollowerHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    auto node = interface->GetNode();

    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, m_topic_name)) {
        return false;
    }

    m_subscription = node->create_subscription<geometry_msgs::msg::Vector3>(
        m_topic_name, 10, std::bind(&ChROSViperWaypointFollowerHandler::Callback, this, _1));

    return true;
}

void ChROSViperWaypointFollowerHandler::Callback(const geometry_msgs::msg::Vector3& msg) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_target_x = msg.x;
    m_target_y = msg.y;
    m_target_z = msg.z;
}

void ChROSViperWaypointFollowerHandler::Tick(double time) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_driver->SetTarget(m_target_x, m_target_y, m_target_z);
}

}  // namespace ros
}  // namespace chrono
