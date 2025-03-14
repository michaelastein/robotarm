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
TmcmBb4CanOpen.hpp

This file contains the definitions for TMCM-BB4 (CANopen).
*/

#ifndef TmcmBb4CanOpen_hpp
#define TmcmBb4CanOpen_hpp

#include <adi_tmcm_bb4/Tmcm1637CanOpen.hpp>

#include <cstdint>
#include <map>
#include <cmath>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "moveit_msgs/msg/motion_plan_request.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

#include <QObject>
#include <QString>


//************************************************************************
// Class for handling TMCM-BB4 on ROS2, using CANopen
//************************************************************************
class TmcmBb4CanOpen : public QObject, public rclcpp::Node
{
    Q_OBJECT

    //************************************************************************
    // constants and types
    //************************************************************************
    private:
        static const std::string NODE_NAME;                 //!< ROS2 node name

        static const std::map<int8_t, std::string> GOAL_STATUS_NAMES;   //!< GoalStatus names

        static const uint8_t TMCM1637_SLOTS = 4;            //!< number of motion controller slots

        static constexpr double TWO_PI = 8.0 * atan( 1.0 ); //!< 2*PI

        static constexpr double JOINT_REDUCTION = 48.0;     //!< joint reduction ratio


    //************************************************************************
    // functions
    //************************************************************************
    public:
        TmcmBb4CanOpen
            (
            bool aResetHomePosition = false     //!< home position reset
            );

        ~TmcmBb4CanOpen();

        static TmcmBb4CanOpen& getInstance()
        {
            static TmcmBb4CanOpen instance;
            return instance;
        }

    private:
        static void absPosTopicCallback();

        void checkMotionControllers();

        static int32_t convertAngleRadToPosition
            (
            const double aAngleRad  //!< joint angle [rad]
            );

        static double convertRadToDeg
            (
            const double aAngle     //!< angle [rad]
            );

        static void executeTopicCallback
            (
            action_msgs::msg::GoalStatusArray::UniquePtr aMessage   //!< ROS2 message
            );

        void handleCanConnect();

        static void handleCanDisconnect();

        void handleCanReset();

        static void handleSigint
            (
            int aSignalNumber   //!< signal number
            );

        static void jointStatesTopicCallback
            (
            sensor_msgs::msg::JointState::UniquePtr aMessage //!< ROS2 message
            );

        static void planTopicCallback
            (
            moveit_msgs::msg::MotionPlanRequest::UniquePtr aMessage //!< ROS2 message
            );

    private slots:
        void updateCanConnectStatus
            (
            const bool aStatus      //!< true if connected
            );

        void updateCanConnectionStr
            (
            const QString aString   //!< string
            );


    //************************************************************************
    // variables
    //************************************************************************
    private:
        rclcpp::Subscription<moveit_msgs::msg::MotionPlanRequest>::SharedPtr    mPlanSubscription;          //!< plan subscription
        rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr      mExecuteSubscription;       //!< execute subscription
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr           mJointStatesSubscription;   //!< joint states subscription

        static rclcpp::Publisher<std_msgs::msg::String>::SharedPtr              sAbsPosPublisher;           //!< absolute position publisher

        rclcpp::TimerBase::SharedPtr            mAbsPosTimer;           //!< absolute position timer

        static bool                             sResetHomePosition;     //!< home position reset
        static CanOpen*                         sCanOpen;               //!< CANopen instance
        static std::vector<Tmcm1637CanOpen>     sTmcm1637Vec;           //!< motion controllers

        static std::vector<bool>                sTmcmIdentityVec;       //!< identity vector
        static std::vector<bool>                sTmcmStateVec;          //!< state vector
        static std::vector<bool>                sTmcmOperEnVec;         //!< operation enabled vector

        static std::vector<int32_t>             sTmcmCrtPosVec;         //!< current positions vector
        static std::vector<int32_t>             sTmcmTargetPosVec;      //!< target positions vector
        static std::vector<int32_t>             sTmcmOffsetPosVec;      //!< offset positions vector

        static std::vector<double>              sTargetJointAngleRadVec;//!< target joint angles [rad] vector
        static std::vector<double>              sSensorJointAngleRadVec;//!< sensor joint angles [rad] vector

        static bool                             sIsExecuting;           //!< if execution is active
};

#endif // TmcmBb4CanOpen_hpp
