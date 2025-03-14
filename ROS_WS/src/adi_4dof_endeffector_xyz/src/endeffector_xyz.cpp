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
endeffector_xyz.cpp

This file contains the sources for the 4DOF robot arm end-effector ROS2 publisher.
*/

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"


//************************************************************************
// Class for a ROS2 publisher for the 4DOF robot arm end-effector
//************************************************************************
class Adi4DofEndeffectorPublisher : public rclcpp::Node
{
    public:
        Adi4DofEndeffectorPublisher
            (
            std::string aClientId,  //!< client_id, unique per topic
            double      aX,         //!< end-effector x coordinate
            double      aY,         //!< end-effector y coordinate
            double      aZ          //!< end-effector z coordinate
            )
            : Node( "adi_4dof_endeffector_publisher" )
        {
            const std::string TOPIC_NAME = "/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback";
            const rclcpp::QoS TOPIC_QOS = rclcpp::QoS( 10 );
            mPublisher = this->create_publisher<visualization_msgs::msg::InteractiveMarkerFeedback>( TOPIC_NAME, TOPIC_QOS );

            geometry_msgs::msg::Point eePoint;
            eePoint.x = aX;
            eePoint.y = aY;
            eePoint.z = aZ;

            geometry_msgs::msg::Pose eePose;
            eePose.position = eePoint;

            visualization_msgs::msg::InteractiveMarkerFeedback eeImf;
            eeImf.pose = eePose;

            eeImf.client_id = aClientId;
            eeImf.header.frame_id = "base_link";
            eeImf.marker_name = "EE:goal_end_effector_link";
            eeImf.control_name = "move";
            eeImf.mouse_point_valid = true;

            // KEEP_ALIVE=0, POSE_UPDATE=1, MENU_SELECT=2, BUTTON_CLICK=3, MOUSE_DOWN=4, MOUSE_UP=5
            eeImf.event_type = visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE;
            this->mPublisher->publish( eeImf );
        }

    private:
        rclcpp::Publisher<visualization_msgs::msg::InteractiveMarkerFeedback>::SharedPtr mPublisher;     //!< publisher
};


int main
    (
    int     argc,
    char*   argv[]
    )
{
    setvbuf( stdout, NULL, _IONBF, BUFSIZ );
    rclcpp::init( argc, argv );

    if( 5 == argc )
    {
        std::string clientId = argv[1];

        double x = atof( argv[2] );
        double y = atof( argv[3] );
        double z = atof( argv[4] );

        auto publisher = std::make_shared<Adi4DofEndeffectorPublisher>( clientId, x, y, z );
        rclcpp::spin( publisher );
    }
    else
    {
        std::cout << "The syntax is:\n" << argv[0] << " client_id x y z\n";
    }

    rclcpp::shutdown();
    return 0;
}
