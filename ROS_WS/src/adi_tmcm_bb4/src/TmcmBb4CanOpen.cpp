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
TmcmBb4CanOpen.cpp

This file contains the sources for the TMCM-BB4 CANopen motion control.
*/

#include <adi_tmcm_bb4/TmcmBb4CanOpen.hpp>

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>


const std::string TmcmBb4CanOpen::NODE_NAME = "tmcm_bb4_canopen";

const std::map<int8_t, std::string> TmcmBb4CanOpen::GOAL_STATUS_NAMES =
{
    { action_msgs::msg::GoalStatus::STATUS_UNKNOWN,   "Unknown"   },
    { action_msgs::msg::GoalStatus::STATUS_ACCEPTED,  "Accepted"  },
    { action_msgs::msg::GoalStatus::STATUS_EXECUTING, "Executing" },
    { action_msgs::msg::GoalStatus::STATUS_CANCELING, "Canceling" },
    { action_msgs::msg::GoalStatus::STATUS_SUCCEEDED, "Succeeded" },
    { action_msgs::msg::GoalStatus::STATUS_CANCELED,  "Canceled"  },
    { action_msgs::msg::GoalStatus::STATUS_ABORTED,   "Aborted"   }
};

bool TmcmBb4CanOpen::sResetHomePosition = false;
CanOpen* TmcmBb4CanOpen::sCanOpen = nullptr;

std::vector<Tmcm1637CanOpen> TmcmBb4CanOpen::sTmcm1637Vec;

std::vector<bool> TmcmBb4CanOpen::sTmcmIdentityVec;
std::vector<bool> TmcmBb4CanOpen::sTmcmStateVec;
std::vector<bool> TmcmBb4CanOpen::sTmcmOperEnVec;

std::vector<int32_t> TmcmBb4CanOpen::sTmcmCrtPosVec;
std::vector<int32_t> TmcmBb4CanOpen::sTmcmTargetPosVec;
std::vector<int32_t> TmcmBb4CanOpen::sTmcmOffsetPosVec;

std::vector<double> TmcmBb4CanOpen::sTargetJointAngleRadVec;
std::vector<double> TmcmBb4CanOpen::sSensorJointAngleRadVec;

rclcpp::Publisher<std_msgs::msg::String>::SharedPtr TmcmBb4CanOpen::sAbsPosPublisher = nullptr;

bool TmcmBb4CanOpen::sIsExecuting = false;

//!************************************************************************
//! Constructor
//!************************************************************************
TmcmBb4CanOpen::TmcmBb4CanOpen
    (
    bool aResetHomePosition     //!< home position reset
    )
    : Node( NODE_NAME )
    , mPlanSubscription( nullptr )
    , mExecuteSubscription( nullptr )
{
    sResetHomePosition = aResetHomePosition;
    std::signal( SIGINT, TmcmBb4CanOpen::handleSigint );
    sCanOpen = CanOpen::getInstance();

    // CANopen signals
    connect( sCanOpen, &CanOpen::connectionStatusChanged, this, &TmcmBb4CanOpen::updateCanConnectStatus );
    connect( sCanOpen, &CanOpen::connectionStrChanged, this, &TmcmBb4CanOpen::updateCanConnectionStr );

    // motion controller boards
    sTmcm1637Vec.resize( TMCM1637_SLOTS );

    sTmcmIdentityVec.resize( TMCM1637_SLOTS );
    sTmcmStateVec.resize( TMCM1637_SLOTS );
    sTmcmOperEnVec.resize( TMCM1637_SLOTS );

    sTmcmCrtPosVec.resize( TMCM1637_SLOTS );
    sTmcmTargetPosVec.resize( TMCM1637_SLOTS );
    sTmcmOffsetPosVec.resize( TMCM1637_SLOTS );

    sTargetJointAngleRadVec.resize( TMCM1637_SLOTS );
    sSensorJointAngleRadVec.resize( TMCM1637_SLOTS );

    for( size_t i = 0; i < sTmcm1637Vec.size(); i++ )
    {
        sTmcm1637Vec.at( i ).setNodeId( i + 1 );

        sTmcmCrtPosVec.at( i ) = 0;
        sTmcmTargetPosVec.at( i ) = 0;
        sTmcmOffsetPosVec.at( i ) = 0;

        sTargetJointAngleRadVec.at( i ) = 0;
        sSensorJointAngleRadVec.at( i ) = 0;
    }

    RCLCPP_INFO( this->get_logger(), "Using joint reduction ratios of %lg:1", JOINT_REDUCTION );

    // CANopen connect and initialize
    handleCanReset();
    handleCanConnect();

    //*********************
    // LISTENERS
    //*********************
    // motion Plan
    const std::string PLAN_TOPIC_NAME = "/motion_plan_request";
    const rclcpp::QoS PLAN_TOPIC_QOS = rclcpp::QoS( 10 );
    mPlanSubscription = this->create_subscription<moveit_msgs::msg::MotionPlanRequest>( PLAN_TOPIC_NAME, PLAN_TOPIC_QOS, planTopicCallback );

    // motion Execute
    const std::string EXECUTE_TOPIC_NAME = "/execute_trajectory/_action/status";
    const rclcpp::QoS EXECUTE_TOPIC_QOS = rclcpp::QoS( 10 );
    mExecuteSubscription = this->create_subscription<action_msgs::msg::GoalStatusArray>( EXECUTE_TOPIC_NAME, EXECUTE_TOPIC_QOS, executeTopicCallback );

    const std::string JOINT_STATES_TOPIC_NAME = "/joint_states";
    const rclcpp::QoS JOINT_STATES_TOPIC_QOS = rclcpp::QoS( 10 );
    mJointStatesSubscription = this->create_subscription<sensor_msgs::msg::JointState>( JOINT_STATES_TOPIC_NAME, JOINT_STATES_TOPIC_QOS, jointStatesTopicCallback );

    //*********************
    // PUBLISHERS
    //*********************
    // BLDC absolute positions
    const uint16_t TIMER_MS = 1000;
    const rclcpp::QoS ABS_POS_TOPIC_QOS = rclcpp::QoS( 10 );
    sAbsPosPublisher = this->create_publisher<std_msgs::msg::String>( "tmcm_bb4_abs_pos", ABS_POS_TOPIC_QOS );
    mAbsPosTimer = this->create_wall_timer( std::chrono::milliseconds( TIMER_MS ), absPosTopicCallback );
}


//!************************************************************************
//! Destructor
//!************************************************************************
TmcmBb4CanOpen::~TmcmBb4CanOpen()
{
    handleCanDisconnect();
}


//!************************************************************************
//! Absolute position publisher callback function
//!
//! @returns: nothing
//!************************************************************************
void TmcmBb4CanOpen::absPosTopicCallback()
{
    if( sIsExecuting )
    {
        auto message = std_msgs::msg::String();
        int32_t crtPosition = 0;
        std::string topicStr;
        bool status = true;
        const size_t VEC_LEN = sTmcm1637Vec.size();

        if( sCanOpen )
        {
            if( sCanOpen->isConnected() )
            {
                rclcpp::Logger logger = rclcpp::get_logger( NODE_NAME );

                for( size_t i = 0; i < VEC_LEN; i++ )
                {
                    switch( i )
                    {
                        case 0:
                            topicStr += "base=";
                            break;

                        case 1:
                            topicStr += "shoulder=";
                            break;

                        case 2:
                            topicStr += "elbow=";
                            break;

                        case 3:
                            topicStr += "wrist=";
                            break;

                        default:
                            break;
                    }

                    if( sTmcm1637Vec.at( i ).getCurrentPosition( crtPosition ) )
                    {
                        sTmcmCrtPosVec.at( i ) = crtPosition;                        

                        if( sResetHomePosition )
                        {
                            sTmcmOffsetPosVec.at( i ) = sTmcmCrtPosVec.at( i );
                        }
                        else
                        {
                            sTmcmCrtPosVec.at( i ) -= sTmcmOffsetPosVec.at( i );
                        }

                        topicStr += std::to_string( crtPosition );

                        if( i < VEC_LEN - 1 )
                        {
                            topicStr += ", ";
                        }
                    }
                    else
                    {
                        status = false;
                        topicStr += "?";

                        if( i < VEC_LEN - 1 )
                        {
                            topicStr += ", ";
                        }
                    }
                }

                if( sResetHomePosition )
                {
                    sResetHomePosition = false;
                    RCLCPP_INFO( logger, "Home positions were reset." );
                }

                message.data = topicStr;
                sAbsPosPublisher->publish( message );

                if( !status )
                {
                    RCLCPP_WARN( logger, "Failed getting the absolute positions from all TMCM-1637" );
                }
            }
        }
    }
}


//!************************************************************************
//! Check the motion controller boards
//!
//! @returns: nothing
//!************************************************************************
void TmcmBb4CanOpen::checkMotionControllers()
{
    rclcpp::Logger logger = rclcpp::get_logger( NODE_NAME );

    if( sCanOpen->isConnected() )
    {
        bool troubleFound = false;
        QString msg;

        //***********************************************************
        // init
        //***********************************************************
        for( size_t i = 0; i < sTmcm1637Vec.size(); i++ )
        {
            sTmcmIdentityVec.at( i ) = false;
            sTmcmStateVec.at( i ) = false;
            sTmcmOperEnVec.at( i ) = false;
            sTmcm1637Vec.at( i ).clearMotorParametersStatus();
        }

        //***********************************************************
        // presence check
        //***********************************************************
        troubleFound = false;

        for( size_t i = 0; i < sTmcm1637Vec.size(); i++ )
        {
            sTmcmIdentityVec.at( i ) = sTmcm1637Vec.at( i ).isIdentityCorrect();

            if( !sTmcmIdentityVec.at( i ) )
            {
                troubleFound = true;
            }
        }

        if( !troubleFound )
        {
            RCLCPP_INFO( logger, "All TMCM-1637 modules found" );
        }
        else
        {
            msg = "Could not find TMCM-1637 modules with node ID: ";
            bool haveFirst = false;

            for( size_t i = 0; i < sTmcm1637Vec.size(); i++ )
            {
                if( !sTmcmIdentityVec.at( i ) )
                {
                    if( haveFirst )
                    {
                        msg += ", ";
                    }

                    msg += QString::number( sTmcm1637Vec.at( i ).getNodeId() );
                    haveFirst = true;
                }
            }

            RCLCPP_ERROR( logger, "%s", msg.toStdString().c_str() );
        }

        //***********************************************************
        // set state to SWITCH_ON_DISABLED
        //***********************************************************
        troubleFound = false;

        for( size_t i = 0; i < sTmcm1637Vec.size(); i++ )
        {
            if( sTmcmIdentityVec.at( i ) )
            {
                Tmcm1637CanOpen::MotionControllerState state = Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_UNKNOWN;
                bool status = sTmcm1637Vec.at( i ).getState( state );

                if( status )
                {
                    sTmcmOperEnVec.at( i ) = ( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_OPERATION_ENABLED == state );

                    if( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_SWITCH_ON_DISABLED != state )
                    {
                        status = sTmcm1637Vec.at( i ).setState( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_SWITCH_ON_DISABLED );
                        sTmcmStateVec.at( i ) = status;

                        if( status )
                        {
                            sTmcmOperEnVec.at( i ) = false;
                        }
                        else
                        {
                            troubleFound = true;
                        }
                    }
                    else
                    {
                        sTmcmStateVec.at( i ) = true;
                    }
                }
                else
                {
                    sTmcmStateVec.at( i ) = false;
                    troubleFound = true;
                }
            }
        }

        if( troubleFound )
        {
            msg = "Could not set state to SWITCH_ON_DISABLED for TMCM-1637 modules with node ID: ";
            bool haveFirst = false;

            for( size_t i = 0; i < sTmcm1637Vec.size(); i++ )
            {
                if( !sTmcmStateVec.at( i ) )
                {
                    if( haveFirst )
                    {
                        msg += ", ";
                    }

                    msg += QString::number( sTmcm1637Vec.at( i ).getNodeId() );
                    haveFirst = true;
                }
            }

            RCLCPP_ERROR( logger, "%s", msg.toStdString().c_str() );
        }

        //***********************************************************
        // set motor parameters
        //***********************************************************
        troubleFound = false;

        for( size_t i = 0; i < sTmcm1637Vec.size(); i++ )
        {
            if( sTmcmIdentityVec.at( i ) &&  sTmcmStateVec.at( i ) )
            {
                if( !sTmcm1637Vec.at( i ).setMotorParameters() )
                {
                    troubleFound = true;
                }
            }
        }

        if( !troubleFound )
        {
            RCLCPP_INFO( logger, "All motor parameters set OK" );
        }
        else
        {
            msg = "Could not set motor parameters for TMCM-1637 modules with node ID: ";
            bool haveFirst = false;

            for( size_t i = 0; i < sTmcm1637Vec.size(); i++ )
            {
                if( !sTmcm1637Vec.at( i ).areMotorParametersSet() )
                {
                    if( haveFirst )
                    {
                        msg += ", ";
                    }

                    msg += QString::number( sTmcm1637Vec.at( i ).getNodeId() );
                    haveFirst = true;
                }
            }

            RCLCPP_ERROR( logger, "%s", msg.toStdString().c_str() );
        }

        //***********************************************************
        // set state to SWITCHED_ON
        //***********************************************************
        troubleFound = false;

        for( size_t i = 0; i < sTmcm1637Vec.size(); i++ )
        {
            if( sTmcmIdentityVec.at( i ) && sTmcmStateVec.at( i ) )
            {
                Tmcm1637CanOpen::MotionControllerState state = Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_UNKNOWN;
                bool status = sTmcm1637Vec.at( i ).getState( state );

                if( status )
                {
                    sTmcmOperEnVec.at( i ) = ( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_OPERATION_ENABLED == state );

                    if( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_SWITCHED_ON != state )
                    {
                        status = sTmcm1637Vec.at( i ).setState( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_SWITCHED_ON );

                        if( !status )
                        {
                            status = sTmcm1637Vec.at( i ).setState( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_SWITCHED_ON );
                        }

                        sTmcmStateVec.at( i ) = status;

                        if( status )
                        {
                            sTmcmOperEnVec.at( i ) = false;
                        }
                        else
                        {
                            troubleFound = true;
                        }
                    }
                }
                else
                {
                    sTmcmStateVec.at( i ) = false;
                    troubleFound = true;
                }                
            }
        }

        if( troubleFound )
        {
            msg = "Could not set state to SWITCHED_ON for TMCM-1637 modules with node ID: ";
            bool haveFirst = false;

            for( size_t i = 0; i < sTmcm1637Vec.size(); i++ )
            {
                if( !sTmcmStateVec.at( i ) )
                {
                    if( haveFirst )
                    {
                        msg += ", ";
                    }

                    msg += QString::number( sTmcm1637Vec.at( i ).getNodeId() );
                    haveFirst = true;
                }
            }

            RCLCPP_ERROR( logger, "%s", msg.toStdString().c_str() );
        }

        //***********************************************************
        // set operating modes
        //***********************************************************
        troubleFound = false;
        Tmcm1637CanOpen::OperatingMode operMode = Tmcm1637CanOpen::OPERATING_MODE_PROFILE_VELOCITY;

        for( size_t i = 0; i < sTmcm1637Vec.size(); i++ )
        {
            if( sTmcm1637Vec.at( i ).areMotorParametersSet() )
            {                
                if( !sTmcm1637Vec.at( i ).setOperatingMode( operMode ) )
                {
                    RCLCPP_ERROR( logger, "Failed setting operating mode to TMCM-1637 with node ID %d", sTmcm1637Vec.at( i ).getNodeId() );
                }

                if( !sTmcm1637Vec.at( i ).setTargetVelocity( 0 ) )
                {
                    RCLCPP_ERROR( logger, "Failed setting velocity to TMCM-1637 with node ID %d", sTmcm1637Vec.at( i ).getNodeId() );
                }
            }
        }
    }
}


//!************************************************************************
//! Convert a joint angle [rad] into an encoded position [-]
//!
//! @returns: nothing
//!************************************************************************
int32_t TmcmBb4CanOpen::convertAngleRadToPosition
    (
    const double aAngleRad  //!< joint angle [rad]
    )
{
    double angleRatio = JOINT_REDUCTION * aAngleRad / TWO_PI;
    int32_t targetPosition = angleRatio * Tmcm1637CanOpen::FULL_ROTATION_ENCODED;
    const uint8_t BLDC_POLE_PAIRS = 8;
    const double ENCODER_INCREMENT = static_cast<double>( Tmcm1637CanOpen::FULL_ROTATION_ENCODED ) / ( 3 * BLDC_POLE_PAIRS );
    int32_t q = targetPosition / ENCODER_INCREMENT;
    int32_t retVal = q * ENCODER_INCREMENT;

    return retVal;
}


//!************************************************************************
//! Convert an angle from [rad] to [deg]
//!
//! @returns: the angle in degrees
//!************************************************************************
double TmcmBb4CanOpen::convertRadToDeg
    (
    const double aAngle     //!< angle [rad]
    )
{
    return aAngle * 360.0 / TWO_PI;
}


//!************************************************************************
//! Motion execute listener callback function
//!
//! @returns: nothing
//!************************************************************************
void TmcmBb4CanOpen::executeTopicCallback
    (
    action_msgs::msg::GoalStatusArray::UniquePtr aMessage   //!< ROS2 message
    )
{
    rclcpp::Logger logger = rclcpp::get_logger( NODE_NAME );
    int8_t goalStatus = action_msgs::msg::GoalStatus::STATUS_UNKNOWN;
    size_t len = aMessage.get()->status_list.size();

    if( len >= 1 )
    {
        goalStatus = aMessage.get()->status_list[len - 1].status;
    }

    RCLCPP_INFO( logger, "Execute status = %s", GOAL_STATUS_NAMES.at( goalStatus ).c_str() );

    sIsExecuting = false;

    switch( goalStatus )
    {
        case action_msgs::msg::GoalStatus::STATUS_EXECUTING:    // 2
            // motion active
            sIsExecuting = true;

            if( sCanOpen )
            {
                if( sCanOpen->getCanDevice() )
                {
                    std::vector<double> angleDiff;
                    angleDiff.resize( TMCM1637_SLOTS );

                    for( size_t i = 0; i < sTmcm1637Vec.size(); i++ )
                    {
                        angleDiff.at( i ) = sSensorJointAngleRadVec.at( i ) - sTargetJointAngleRadVec.at( i );
                    }

                    RCLCPP_INFO( logger, "Execute data:" );
                    RCLCPP_INFO( logger, "- base_diff = %.2lf [deg]", convertRadToDeg( angleDiff.at( 0 ) ) );
                    RCLCPP_INFO( logger, "- shoulder_diff = %.2lf [deg]", convertRadToDeg( angleDiff.at( 1 ) ) );
                    RCLCPP_INFO( logger, "- elbow_diff = %.2lf [deg]", convertRadToDeg( angleDiff.at( 2 ) ) );
                    RCLCPP_INFO( logger, "- wrist_diff = %.2lf [deg]", convertRadToDeg( angleDiff.at( 3 ) ) );

                    double minAngleDiff = *min_element( angleDiff.begin(), angleDiff.end() );
                    double maxAngleDiff = *max_element( angleDiff.begin(), angleDiff.end() );
                    double maxAbs = std::max( fabs( minAngleDiff ) , fabs( maxAngleDiff ) );

                    for( size_t i = 0; i < sTmcm1637Vec.size(); i++ )
                    {
                        int32_t crtVelRpm = Tmcm1637CanOpen::POSITION_MODE_VELOCITY_RPM * angleDiff.at( i ) / maxAbs;
                        bool status = sTmcm1637Vec.at( i ).setTargetVelocity( crtVelRpm );

                        if( !status )
                        {
                            status = sTmcm1637Vec.at( i ).setTargetVelocity( crtVelRpm );
                        }

                        if( !status )
                        {
                            RCLCPP_ERROR( logger, "Failed setting velocity to TMCM-1637 with node ID %d", sTmcm1637Vec.at( i ).getNodeId() );
                        }

                        bool isOperationEnabled = sTmcmOperEnVec.at( i );
                        Tmcm1637CanOpen::MotionControllerState state = Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_UNKNOWN;
                        status = sTmcm1637Vec.at( i ).getState( state );

                        if( status )
                        {
                            isOperationEnabled = ( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_OPERATION_ENABLED == state );
                        }

                        if( !isOperationEnabled )
                        {
                            status = sTmcm1637Vec.at( i ).setState( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_OPERATION_ENABLED );

                            if( !status )
                            {
                                status = sTmcm1637Vec.at( i ).setState( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_OPERATION_ENABLED );
                            }

                            if( status )
                            {
                                sTmcmOperEnVec.at( i ) = true;
                            }
                            else
                            {
                                RCLCPP_ERROR( logger, "Failed setting OPERATION_ENABLED state to TMCM-1637 with node ID %d", sTmcm1637Vec.at( i ).getNodeId() );
                            }
                        }
                    }
                }
            }
            break;

        case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:    // 4
            // motion stopped, end position achieved
        case action_msgs::msg::GoalStatus::STATUS_CANCELING:    // 3
        case action_msgs::msg::GoalStatus::STATUS_CANCELED:     // 5
        case action_msgs::msg::GoalStatus::STATUS_ABORTED:      // 6
            // forced stop
            if( sCanOpen )
            {
                if( sCanOpen->getCanDevice() )
                {
                    for( size_t i = 0; i < sTmcm1637Vec.size(); i++ )
                    {
                        bool status = sTmcm1637Vec.at( i ).setTargetVelocity( 0 );

                        if( !status )
                        {
                            sTmcm1637Vec.at( i ).setTargetVelocity( 0 );
                        }

                        bool isSwitchedOn = !sTmcmOperEnVec.at( i );
                        Tmcm1637CanOpen::MotionControllerState state = Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_UNKNOWN;
                        status = sTmcm1637Vec.at( i ).getState( state );

                        if( status )
                        {
                            isSwitchedOn = ( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_SWITCHED_ON == state );
                        }

                        if( !isSwitchedOn )
                        {
                            status = sTmcm1637Vec.at( i ).setState( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_SWITCHED_ON );

                            if( !status )
                            {
                                status = sTmcm1637Vec.at( i ).setState( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_SWITCHED_ON );
                            }

                            if( status )
                            {
                                sTmcmOperEnVec.at( i ) = false;
                            }
                            else
                            {
                                RCLCPP_ERROR( logger, "Failed setting SWITCHED_ON state to TMCM-1637 with node ID %d", sTmcm1637Vec.at( i ).getNodeId() );
                            }
                        }
                    }
                }
            }
            break;

        case action_msgs::msg::GoalStatus::STATUS_UNKNOWN:      // 0
        case action_msgs::msg::GoalStatus::STATUS_ACCEPTED:     // 1
        default:
            // intentionally do nothing
            break;
    }
}


//!************************************************************************
//! Handle for CAN connect
//!
//! @returns: nothing
//!************************************************************************
void TmcmBb4CanOpen::handleCanConnect()
{
    rclcpp::Logger logger = rclcpp::get_logger( NODE_NAME );

    if( sCanOpen )
    {
        RCLCPP_INFO( logger, "Connecting to CAN bus.." );
        bool status = sCanOpen->initCanBus();
        RCLCPP_INFO( logger, status ? "CAN bus connected." : "CAN bus NOT connected." );

        checkMotionControllers();
    }
    else
    {
        RCLCPP_INFO( logger, "CANopen instance is nullptr." );
    }
}


//!************************************************************************
//! Handle for CAN disconnect event
//!
//! @returns: nothing
//!************************************************************************
void TmcmBb4CanOpen::handleCanDisconnect()
{
    if( sCanOpen )
    {
        if( sCanOpen->getCanDevice() )
        {
            if( sCanOpen->isConnected() )
            {
                for( size_t i = 0; i < sTmcm1637Vec.size(); i++ )
                {
                    if( sTmcmIdentityVec.at( i ) )
                    {
                        sTmcm1637Vec.at( i ).setState( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_SWITCH_ON_DISABLED );
                    }
                }

                sCanOpen->disconnect();

                rclcpp::Logger logger = rclcpp::get_logger( NODE_NAME );
                RCLCPP_INFO( logger, "Disconnected from CAN bus." );
            }
        }
    }
}


//!************************************************************************
//! Handle for CAN reset
//!
//! @returns: nothing
//!************************************************************************
void TmcmBb4CanOpen::handleCanReset()
{
    if( sCanOpen )
    {
        sCanOpen->reset();

        rclcpp::Logger logger = rclcpp::get_logger( NODE_NAME );
        RCLCPP_INFO( logger, "CAN bus controller was reset" );
    }
}


//!************************************************************************
//! SIGINT handler
//!
//! @returns: nothing
//!************************************************************************
void TmcmBb4CanOpen::handleSigint
    (
    int aSignalNumber   //!< signal number
    )
{
    Q_UNUSED( aSignalNumber );
    handleCanDisconnect();
    exit( 0 );
}


//!************************************************************************
//! Joint states listener callback function
//!
//! @returns: nothing
//!************************************************************************
void TmcmBb4CanOpen::jointStatesTopicCallback
    (
    sensor_msgs::msg::JointState::UniquePtr aMessage //!< ROS2 message
    )
{
    for( size_t i = 0; i < sSensorJointAngleRadVec.size(); i++ )
    {
        sSensorJointAngleRadVec.at( i ) = aMessage.get()->position[i];
    }
}


//!************************************************************************
//! Motion plan listener callback function
//!
//! @returns: nothing
//!************************************************************************
void TmcmBb4CanOpen::planTopicCallback
    (
    moveit_msgs::msg::MotionPlanRequest::UniquePtr aMessage //!< ROS2 message
    )
{
    for( size_t i = 0; i < sTargetJointAngleRadVec.size(); i++ )
    {
        sTargetJointAngleRadVec.at( i ) = aMessage.get()->goal_constraints[0].joint_constraints[i].position;
        sTmcmTargetPosVec.at( i ) = convertAngleRadToPosition( sTargetJointAngleRadVec.at( i ) );
    }

    rclcpp::Logger logger = rclcpp::get_logger( NODE_NAME );

    RCLCPP_INFO( logger, "Plan data:" );

    RCLCPP_INFO( logger, "- base = %.2lf [deg], pos = %d",
                 convertRadToDeg( sTargetJointAngleRadVec.at( 0 ) ), sTmcmTargetPosVec.at( 0 ) );

    RCLCPP_INFO( logger, "- shoulder = %.2lf [deg], pos = %d",
                 convertRadToDeg( sTargetJointAngleRadVec.at( 1 ) ), sTmcmTargetPosVec.at( 1 ) );

    RCLCPP_INFO( logger, "- elbow = %.2lf [deg], pos = %d",
                 convertRadToDeg( sTargetJointAngleRadVec.at( 2 ) ), sTmcmTargetPosVec.at( 2 ) );

    RCLCPP_INFO( logger, "- wrist = %.2lf [deg], pos = %d",
                 convertRadToDeg( sTargetJointAngleRadVec.at( 3 ) ), sTmcmTargetPosVec.at( 3 ) );
}


//!************************************************************************
//! Updates related to CAN connection
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void TmcmBb4CanOpen::updateCanConnectStatus
    (
    const bool aStatus      //!< true if connected
    )
{
    rclcpp::Logger logger = rclcpp::get_logger( NODE_NAME );

    if( aStatus )
    {
        RCLCPP_INFO( logger, "CAN BUS => CONNECTED" );
    }
    else
    {
        RCLCPP_WARN( logger, "CAN BUS => DISCONNECTED" );
    }
}


//!************************************************************************
//! Update the CAN connection string
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void TmcmBb4CanOpen::updateCanConnectionStr
    (
    const QString aString   //!< string
    )
{
    if( aString.size() )
    {
        rclcpp::Logger logger = rclcpp::get_logger( NODE_NAME );
        RCLCPP_INFO( logger, "CAN CONNECTION: %s", aString.toStdString().c_str() );
    }
}
