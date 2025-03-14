///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2025 Mihai Ursu                                                 //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
// (at your option) any later version.                                           //
//                                                                               //
// This program is distributed in the hope that it will be useful,               //
// but WITHOUT ANY WARRANTY; without even the implied warranty of                //
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                  //
// GNU General Public License V3 for more details.                               //
//                                                                               //
// You should have received a copy of the GNU General Public License             //
// along with this program. If not, see <http://www.gnu.org/licenses/>.          //
///////////////////////////////////////////////////////////////////////////////////

/*
endeffector_listener.cpp

This file contains the sources for the 4DOF robot arm end-effector ROS2 listener.
*/

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"


//************************************************************************
// Class for a ROS2 subscriber for the 4DOF robot arm end-effector
//************************************************************************
class Adi4DofEndeffectorSubscriber : public rclcpp::Node
{
    public:
        Adi4DofEndeffectorSubscriber()
            : Node( "adi_4dof_endeffector_subscriber" )
        {
            auto topicCallback = [this]
                                    (
                                    visualization_msgs::msg::InteractiveMarkerFeedback::UniquePtr aMessage
                                    ) -> void
            {
                geometry_msgs::msg::Point point = aMessage.get()->pose.position;
                std::string clientId = aMessage.get()->client_id;

                double eeX = point.x;
                double eeY = point.y;
                double eeZ = point.z;

                RCLCPP_INFO( this->get_logger(), "Received: x = %.3lf, y = %.3lf, z = %.3lf, client_id = %s", eeX, eeY, eeZ, clientId.c_str() );
            };

            const std::string TOPIC_NAME = "/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback";
            const rclcpp::QoS TOPIC_QOS = rclcpp::QoS( 10 );
            mSubscription = this->create_subscription<visualization_msgs::msg::InteractiveMarkerFeedback>( TOPIC_NAME, TOPIC_QOS, topicCallback );
        }

    private:
        rclcpp::Subscription<visualization_msgs::msg::InteractiveMarkerFeedback>::SharedPtr mSubscription;   //!< subscription
};


int main
    (
    int     argc,
    char*   argv[]
    )
{
    setvbuf( stdout, NULL, _IONBF, BUFSIZ );
    rclcpp::init( argc, argv );

    auto subscriber = std::make_shared<Adi4DofEndeffectorSubscriber>();
    rclcpp::spin( subscriber );

    rclcpp::shutdown();
    return 0;
}
