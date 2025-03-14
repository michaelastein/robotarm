///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2024 Mihai Ursu                                                 //
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
adi_4dof_ik_kinematics_plugin.cpp

This file contains the sources for the 4DOF robot inverse kinematic plugin.
*/

#include <adi_4dof_ik/adi_4dof_ik_kinematics_plugin.hpp>

#include <algorithm>
#include <iostream>

#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <moveit/robot_state/robot_state.h>
#include <tf2_kdl/tf2_kdl.hpp>
#include <urdf/model.h>


namespace adi_4dof_ik_kinematics_plugin
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger( "adi_4dof_ik_kinematics_plugin.adi_4dof_ik_kinematics_plugin" );
static ADI_4DOF_KinematicsPlugin::JointAngles gJointAngles;

//!************************************************************************
//! Constructor
//!************************************************************************
ADI_4DOF_KinematicsPlugin::ADI_4DOF_KinematicsPlugin()
    : mIsActive( false )
{
    mRobotArmInstance = SerialRobot::getInstance();

    // Fill the vector with the correct number of links and joints.
    // Their current parameters may have changed => runtime values
    // are read in initialize(), where identification is done by name.
    mRobotPartsVec = mRobotArmInstance->getRobotPartsVec();
}


//!************************************************************************
//! Destructor
//!************************************************************************
ADI_4DOF_KinematicsPlugin::~ADI_4DOF_KinematicsPlugin()
{
}


//!************************************************************************
//! Check if the joint angles are in the current allowed ranges
//!
//! @returns true if all joint angles are in their allowed ranges
//!************************************************************************
bool ADI_4DOF_KinematicsPlugin::checkJointAngles
    (
    double& aTh2,  //!< angle of base joint [rad]
    double& aTh3,  //!< angle of shoulder joint [rad]
    double& aTh4,  //!< angle of elbow joint [rad]
    double& aTh5   //!< angle of wrist joint [rad]
    ) const
{
    bool status = true;

    if( status )
    {
        if( aTh2 >= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_JOINT ).joint.limit.lowerAngleRad - ZERO_RAD_THD
         && aTh2 < mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_JOINT ).joint.limit.lowerAngleRad )
        {
            aTh2 = mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_JOINT ).joint.limit.lowerAngleRad;
        }
        else if( aTh2 < mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_JOINT ).joint.limit.lowerAngleRad - ZERO_RAD_THD )
        {
            status = false;
        }
    }
    if( status )
    {
        if( aTh2 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_JOINT ).joint.limit.upperAngleRad + ZERO_RAD_THD
         && aTh2 > mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_JOINT ).joint.limit.upperAngleRad )
        {
            aTh2 = mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_JOINT ).joint.limit.upperAngleRad;
        }
        else if( aTh2 > mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_JOINT ).joint.limit.upperAngleRad + ZERO_RAD_THD )
        {
            status = false;
        }
    }


    if( status )
    {
        if( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad >= 0 )
        {
            if( aTh3 >= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad - ZERO_RAD_THD
             && aTh3 < mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad )
            {
                aTh3 = mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad;
            }
            else if( aTh3 < mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad - ZERO_RAD_THD )
            {
                status = false;
            }
        }
        else
        {
            if( aTh3 < 0 )
            {
                status = false;
            }
        }
    }
    if( status )
    {
        if( aTh3 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad + ZERO_RAD_THD
         && aTh3 > mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad )
        {
            aTh3 = mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad;
        }
        else if( aTh3 > mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad + ZERO_RAD_THD )
        {
            status = false;
        }
    }


    if( status )
    {
        if( aTh4 >= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad - ZERO_RAD_THD
         && aTh4 < mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad )
        {
            aTh4 = mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad;
        }
        else if( aTh4 < mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad - ZERO_RAD_THD )
        {
            status = false;
        }
    }
    if( status )
    {
        if( aTh4 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad + ZERO_RAD_THD
         && aTh4 > mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad )
        {
            aTh4 = mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad;
        }
        else if( aTh4 > mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad + ZERO_RAD_THD )
        {
            status = false;
        }
    }


    if( status )
    {
        if( aTh5 >= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.lowerAngleRad - ZERO_RAD_THD
         && aTh5 < mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.lowerAngleRad )
        {
            aTh5 = mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.lowerAngleRad;
        }
        else if( aTh5 < mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.lowerAngleRad - ZERO_RAD_THD )
        {
            status = false;
        }
    }
    if( status )
    {
        if( aTh5 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.upperAngleRad + ZERO_RAD_THD
         && aTh5 > mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.upperAngleRad )
        {
            aTh5 = mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.upperAngleRad;
        }
        else if( aTh5 > mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.upperAngleRad + ZERO_RAD_THD )
        {
            status = false;
        }
    }

    return status;
}


//!************************************************************************
//! Compute the inverse kinematic model for 4DOF
//! It is based on cascading a series of approaches that use analytical
//! solutions, so that provided results are exact.
//!
//! @returns true if the solution exists
//!************************************************************************
bool ADI_4DOF_KinematicsPlugin::computeIkm
    (
    const double                aX,             //!< end-effector x coordinate
    const double                aY,             //!< end-effector y coordinate
    const double                aZ,             //!< end-effector z coordinate
    const double                aL1,            //!< length of base link
    const double                aL2,            //!< length of column link
    const double                aL3,            //!< length of upper arm link
    const double                aL4,            //!< length of lower arm link
    const double                aL5,            //!< length of end effector link
    std::vector<JointAngles>&   aJointAnglesVec //!< joint angles vector
    ) const
{
    bool status = false;

    double rank = 0;
    JointAngles angles;
    memset( &angles, 0, sizeof( angles ) );

    aJointAnglesVec.clear();
    double th2 = 0;
    double th3 = 0;
    double th4 = 0;
    double th5 = 0;

    // analytical solution @ th5 = -(th3+th4)
    if( computeIkmHee( aX, aY, aZ,
                       aL1, aL2, aL3, aL4, aL5,
                       th2, th3, th4, th5 ) )
    {
        if( validateIkmSet( th2, th3, th4, th5, aX, aY, aZ ) )
        {
            status = true;
            rank = computeJointAnglesRank( th2, th3, th4, th5 );
            angles = { th2, th3, th4, th5, rank };
            aJointAnglesVec.push_back( angles );
        }
    }

    // analytical solution @ th5 = 0
    if( computeIkmTh5Eq0( aX, aY, aZ,
                          aL1, aL2, aL3, aL4, aL5,
                          th2, th3, th4, th5 ) )
    {
        if( validateIkmSet( th2, th3, th4, th5, aX, aY, aZ ) )
        {
            status = true;
            rank = computeJointAnglesRank( th2, th3, th4, th5 );
            angles = { th2, th3, th4, th5, rank };
            aJointAnglesVec.push_back( angles );
        }
    }

    // analytical solution @ th5 = -th4
    if( computeIkmTh5EqMth4( aX, aY, aZ,
                             aL1, aL2, aL3, aL4, aL5,
                             th2, th3, th4, th5 ) )
    {
        if( validateIkmSet( th2, th3, th4, th5, aX, aY, aZ ) )
        {
            status = true;
            rank = computeJointAnglesRank( th2, th3, th4, th5 );
            angles = { th2, th3, th4, th5, rank };
            aJointAnglesVec.push_back( angles );
        }
    }

    // analytical solution @ th5 = -th3
    if( computeIkmTh5EqMth3( aX, aY, aZ,
                             aL1, aL2, aL3, aL4, aL5,
                             th2, th3, th4, th5 ) )
    {
        if( validateIkmSet( th2, th3, th4, th5, aX, aY, aZ ) )
        {
            status = true;
            rank = computeJointAnglesRank( th2, th3, th4, th5 );
            angles = { th2, th3, th4, th5, rank };
            aJointAnglesVec.push_back( angles );
        }
    }

    // analytical solution @ th4 = -th3
    if( computeIkmTh4EqMth3( aX, aY, aZ,
                             aL1, aL2, aL3, aL4, aL5,
                             th2, th3, th4, th5 ) )
    {
        if( validateIkmSet( th2, th3, th4, th5, aX, aY, aZ ) )
        {
            status = true;
            rank = computeJointAnglesRank( th2, th3, th4, th5 );
            angles = { th2, th3, th4, th5, rank };
            aJointAnglesVec.push_back( angles );
        }
    }

    // analytical solution @ th5+th4 = PI/2
    if( computeIkmTh5Th4EqPi2( aX, aY, aZ,
                               aL1, aL2, aL3, aL4, aL5,
                               th2, th3, th4, th5 ) )
    {
        if( validateIkmSet( th2, th3, th4, th5, aX, aY, aZ ) )
        {
            status = true;
            rank = computeJointAnglesRank( th2, th3, th4, th5 );
            angles = { th2, th3, th4, th5, rank };
            aJointAnglesVec.push_back( angles );
        }
    }

    // analytical solution @ th5+th4 = -PI/2
    if( computeIkmTh5Th4EqMpi2( aX, aY, aZ,
                                aL1, aL2, aL3, aL4, aL5,
                                th2, th3, th4, th5 ) )
    {
        if( validateIkmSet( th2, th3, th4, th5, aX, aY, aZ ) )
        {
            status = true;
            rank = computeJointAnglesRank( th2, th3, th4, th5 );
            angles = { th2, th3, th4, th5, rank };
            aJointAnglesVec.push_back( angles );
        }
    }

    // analytical solution @ th5+th4 = PI or
    //                       th5+th4 = -PI
    if( computeIkmTh5Th4EqPi( aX, aY, aZ,
                              aL1, aL2, aL3, aL4, aL5,
                              th2, th3, th4, th5 ) )
    {
        if( validateIkmSet( th2, th3, th4, th5, aX, aY, aZ ) )
        {
            status = true;
            rank = computeJointAnglesRank( th2, th3, th4, th5 );
            angles = { th2, th3, th4, th5, rank };
            aJointAnglesVec.push_back( angles );
        }
    }

    return status;
}


//!************************************************************************
//! Compute the inverse kinematic model for 4DOF
//! ***  Horizontal end-effector, th5 = -(th3+th4)  ***
//! It provides an exact analytical solution.
//! This function should be called from computeIkm()
//!
//! @returns true if the solution exists
//!************************************************************************
bool ADI_4DOF_KinematicsPlugin::computeIkmHee
    (
    const double aX,    //!< end-effector x coordinate
    const double aY,    //!< end-effector y coordinate
    const double aZ,    //!< end-effector z coordinate
    const double aL1,   //!< length of base link
    const double aL2,   //!< length of column link
    const double aL3,   //!< length of upper arm link
    const double aL4,   //!< length of lower arm link
    const double aL5,   //!< length of end effector link
    double&      aTh2,  //!< angle of base joint [rad]
    double&      aTh3,  //!< angle of shoulder joint [rad]
    double&      aTh4,  //!< angle of elbow joint [rad]
    double&      aTh5   //!< angle of wrist joint [rad]
    ) const
{
    bool status = true;

    aTh2 = atan2( aY, aX );
    refineTh2Angle( aTh2 );

    aTh3 = 0;
    aTh4 = 0;
    aTh5 = 0;

    double th31 = 0;
    double th32 = 0;

    double th41 = 0;
    double th42 = 0;

    double a = 0;
    double b = 0;

    double u1 = 0;
    double u2 = 0;
    double u3 = 0;
    double u4 = 0;

    double v1 = 0;
    double v2 = 0;
    double v3 = 0;
    double v4 = 0;
    double v5 = 0;
    double v6 = 0;
    double v7 = 0;
    double v8 = 0;
    double v9 = 0;
    double v10 = 0;

    if( 0 == sin( aTh2 ) )
    {
        a = aX / cos( aTh2 ) - aL5;
    }
    else
    {
        a = aY / sin( aTh2 ) - aL5;
    }

    b = aZ - aL1 - aL2;

    // th4{1,2}
    u1 = a * a + b * b - aL3 * aL3 - aL4 * aL4;
    u2 =  u1 + 2 * aL3 * aL4;
    u3 = -u1 + 2 * aL3 * aL4;
    u4 = u2 * u3;

    if( u4 >= -ZERO_L_THD && u4 < 0 )
    {
        u4 = 0;
    }

    if( u4 < 0 )
    {
        status = false;
    }

    if( status )
    {
        th41 = -2 * atan2( sqrt( u4 ), u2 );
        th42 = -th41;

        refineAnglePlusMinusPi( th41 );
        refineAnglePlusMinusPi( th42 );
    }

    // th3{1,2}
    if( status )
    {
        v1 = a * a + b * b + aL3 * aL3 - aL4 * aL4 + 2 * a * aL3;
        v2 = a * a + b * b - aL3 * aL3 - aL4 * aL4 + 2 * aL3 * aL4;

        if( 0 == v2 )
        {
            status = false;
        }
    }

    if( status )
    {
        v3 = v2 * ( -a * a - b * b + aL3 * aL3 + aL4 * aL4 + 2 * aL3 * aL4 );

        if( v3 >= -ZERO_L_THD && v3 < 0 )
        {
            v3 = 0;
        }

        if( v3 < 0 )
        {
            status = false;
        }
    }

    if( status )
    {
        v3 = sqrt( v3 );
        double v32 = v3 / v2;
        v4 = aL4 * aL4 * v32;
        v5 = aL3 * aL3 * v32;
        v6 = b * b * v32;
        v7 = a * a * v32;
        v8 = 2 * aL3 * aL4 * v32;
        v9 = v7 + v6 - v5 - v4 + v8;
        v10 = 2 * b * aL3;

        th31 = 2 * atan2( v10 + v9, v1 );
        th32 = 2 * atan2( v10 - v9, v1 );

        refineAnglePlusMinusPi( th31 );
        refineAnglePlusMinusPi( th32 );

        double s1 = th31 + th41;
        double s2 = th32 + th42;

        if( ( ( s1 >= 0 ) && ( s1 <= 0.5 * SerialRobot::PI ) )
         || ( ( s1 >= -0.5 * SerialRobot::PI ) && ( s1 <= 0 ) )
          )
        {
            aTh3 = th31;
            aTh4 = th41;
        }
        else if( ( ( s2 >= 0 ) && ( s2 <= 0.5 * SerialRobot::PI ) )
              || ( ( s2 >= -0.5 * SerialRobot::PI ) && ( s2 <= 0 ) )
               )
        {
            aTh3 = th32;
            aTh4 = th42;
        }
        else
        {
            status = false;
        }

        aTh5 = -( aTh3 + aTh4 );
    }

    if( status )
    {
        status = checkJointAngles( aTh2, aTh3, aTh4, aTh5 );
    }

    return status;
}


//!************************************************************************
//! Compute the inverse kinematic model for 4DOF
//! ***  th5 = 0  ***
//! It provides an exact analytical solution.
//! This function should be called from computeIkm()
//!
//! @returns true if the solution exists
//!************************************************************************
bool ADI_4DOF_KinematicsPlugin::computeIkmTh5Eq0
    (
    const double aX,    //!< end-effector x coordinate
    const double aY,    //!< end-effector y coordinate
    const double aZ,    //!< end-effector z coordinate
    const double aL1,   //!< length of base link
    const double aL2,   //!< length of column link
    const double aL3,   //!< length of upper arm link
    const double aL4,   //!< length of lower arm link
    const double aL5,   //!< length of end effector link
    double&      aTh2,  //!< angle of base joint [rad]
    double&      aTh3,  //!< angle of shoulder joint [rad]
    double&      aTh4,  //!< angle of elbow joint [rad]
    double&      aTh5   //!< angle of wrist joint [rad]
    ) const
{
    bool status = true;

    aTh2 = atan2( aY, aX );
    refineTh2Angle( aTh2 );

    aTh3 = 0;
    aTh4 = 0;
    aTh5 = 0;

    double th31 = 0;
    double th32 = 0;

    double th41 = 0;
    double th42 = 0;

    double a = 0;
    double b = 0;

    double u1 = 0;
    double u2 = 0;
    double u3 = 0;

    double v1 = 0;
    double v2 = 0;
    double v3 = 0;
    double v4 = 0;
    double v5 = 0;
    double v6 = 0;
    double v7 = 0;
    double v8 = 0;
    double v9 = 0;
    double v10 = 0;
    double v11 = 0;
    double v12 = 0;
    double v13 = 0;
    double v14 = 0;
    double v15 = 0;

    aTh5 = 0;

    if( 0 == sin( aTh2 ) )
    {
        a = aX / cos( aTh2 );
    }
    else
    {
        a = aY / sin( aTh2 );
    }

    b = aZ - aL1 - aL2;

    // th4{1,2}
    u1 = a * a + b * b - aL3 * aL3 - aL4 * aL4 - aL5 * aL5 - 2 * aL4 * aL5;
    u2 = 2 * aL3 * ( aL4 + aL5 );
    u3 = ( u1 + u2 ) * ( -u1 + u2 );

    if( u3 >= -ZERO_L_THD && u3 < 0 )
    {
        u3 = 0;
    }

    if( u3 < 0 )
    {
        status = false;
    }

    if( status )
    {
        th41 = 2 * atan2( sqrt( u3 ), u1 + u2 );
        th42 = -th41;

        refineAnglePlusMinusPi( th41 );
        refineAnglePlusMinusPi( th42 );
    }

    // th3{1,2}
    if( status )
    {
        v1 = a * a + b * b + aL3 * aL3 - aL4 * aL4 - aL5 * aL5 - 2 * aL4 * aL5 + 2 * a * aL3;
        v2 = a * a + b * b - aL3 * aL3 - aL4 * aL4 - aL5 * aL5 - 2 * aL4 * aL5;
        v3 = 2 * aL3 * ( aL4 + aL5 );
        v4 = v2 + v3;

        if( 0 == v4 )
        {
            status = false;
        }
    }

    double vprod = v4 * ( -v2 + v3 );

    if( vprod >= -ZERO_L_THD && vprod < 0 )
    {
        vprod = 0;
    }

    if( status )
    {
        if( vprod < 0 )
        {
            status = false;
        }
    }

    if( status )
    {
        v5 = sqrt( vprod );
        double v54 = v5 / v4;
        v6 = aL5 * aL5 * v54;
        v7 = aL4 * aL4 * v54;
        v8 = aL3 * aL3 * v54;
        v9 = b * b * v54;
        v10 = a * a * v54;
        v11 = 2 * aL4 * aL5 * v54;
        v12 = 2 * aL3 * aL5 * v54;
        v13 = 2 * aL3 * aL4 * v54;
        v14 = v10 + v9 - v8 - v7 - v6 + v13 + v12 - v11;
        v15 = 2 * b * aL3;

        th31 = 2 * atan2( v15 - v14, v1 );
        th32 = 2 * atan2( v15 + v14, v1 );

        refineAnglePlusMinusPi( th31 );
        refineAnglePlusMinusPi( th32 );

        if( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad <= th31
            && th31 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad
            &&
            mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad <= th41
            && th41 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad
          )
        {
            aTh3 = th31;
            aTh4 = th41;
        }
        else if( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad <= th32
                 && th32 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad
                 &&
                 mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad <= th42
                 && th42 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad
               )
        {
            aTh3 = th32;
            aTh4 = th42;
        }
        else
        {
            status = false;
        }
    }

    if( status )
    {
        status = checkJointAngles( aTh2, aTh3, aTh4, aTh5 );
    }

    return status;
}


//!************************************************************************
//! Compute the inverse kinematic model for 4DOF
//! ***  th5 = -th4  ***
//! It provides an exact analytical solution.
//! This function should be called from computeIkm()
//!
//! @returns true if the solution exists
//!************************************************************************
bool ADI_4DOF_KinematicsPlugin::computeIkmTh5EqMth4
    (
    const double aX,    //!< end-effector x coordinate
    const double aY,    //!< end-effector y coordinate
    const double aZ,    //!< end-effector z coordinate
    const double aL1,   //!< length of base link
    const double aL2,   //!< length of column link
    const double aL3,   //!< length of upper arm link
    const double aL4,   //!< length of lower arm link
    const double aL5,   //!< length of end effector link
    double&      aTh2,  //!< angle of base joint [rad]
    double&      aTh3,  //!< angle of shoulder joint [rad]
    double&      aTh4,  //!< angle of elbow joint [rad]
    double&      aTh5   //!< angle of wrist joint [rad]
    ) const
{
    bool status = true;

    aTh2 = atan2( aY, aX );
    refineTh2Angle( aTh2 );

    aTh3 = 0;
    aTh4 = 0;
    aTh5 = 0;

    double th31 = 0;
    double th32 = 0;

    double th41 = 0;
    double th42 = 0;

    double th51 = 0;
    double th52 = 0;

    double a = 0;
    double b = 0;

    double u1 = 0;
    double u2 = 0;
    double u3 = 0;

    double v1 = 0;
    double v2 = 0;
    double v3 = 0;
    double v4 = 0;
    double v5 = 0;
    double v6 = 0;
    double v7 = 0;
    double v8 = 0;
    double v9 = 0;
    double v10 = 0;
    double v11 = 0;
    double v12 = 0;
    double v13 = 0;
    double v14 = 0;
    double v15 = 0;

    if( 0 == sin( aTh2 ) )
    {
        a = aX / cos( aTh2 );
    }
    else
    {
        a = aY / sin( aTh2 );
    }

    b = aZ - aL1 - aL2;

    // th4{1,2}
    u1 = a * a + b * b - aL3 * aL3 - aL4 * aL4 - aL5 * aL5 - 2 * aL3 * aL5;
    u2 = 2 * aL4 * ( aL3 + aL5 );
    u3 = ( u1 + u2 ) * ( -u1 + u2 );

    if( u3 >= -ZERO_L_THD && u3 < 0 )
    {
        u3 = 0;
    }

    if( u3 < 0 )
    {
        status = false;
    }

    if( status )
    {
        th41 = 2 * atan2( sqrt( u3 ), u1 + u2 );
        th42 = -th41;

        refineAnglePlusMinusPi( th41 );
        refineAnglePlusMinusPi( th42 );

        th51 = -th41;
        th52 = -th42;
    }

    // th3{1,2}
    if( status )
    {
        v1 = a * a + b * b + aL3 * aL3 - aL4 * aL4 + aL5 * aL5 + 2 * aL3 * aL5 + 2 * a * ( aL3 + aL5 );
        v2 = a * a + b * b - aL3 * aL3 - aL4 * aL4 - aL5 * aL5 - 2 * aL3 * aL5;
        v3 = 2 * aL4 * ( aL3 + aL5 );
        v4 = v2 + v3;

        if( 0 == v4 )
        {
            status = false;
        }
    }

    double vprod = v4 * ( -v2 + v3 );

    if( vprod >= -ZERO_L_THD && vprod < 0 )
    {
        vprod = 0;
    }

    if( status )
    {
        if( vprod < 0 )
        {
            status = false;
        }
    }

    if( status )
    {
        v5 = sqrt( vprod );
        double v54 = v5 / v4;
        v6 = aL5 * aL5 * v54;
        v7 = aL4 * aL4 * v54;
        v8 = aL3 * aL3 * v54;
        v9 = b * b * v54;
        v10 = a * a * v54;
        v11 = 2 * aL4 * aL5 * v54;
        v12 = 2 * aL3 * aL5 * v54;
        v13 = 2 * aL3 * aL4 * v54;
        v14 = v10 + v9 - v8 - v7 - v6 + v13 + v12 - v11;
        v15 = 2 * b * ( aL3 + aL5 );

        th31 = 2 * atan2( v15 - v14, v1 );
        th32 = 2 * atan2( v15 + v14, v1 );

        refineAnglePlusMinusPi( th31 );
        refineAnglePlusMinusPi( th32 );

        if( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad <= th31
            && th31 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad
            &&
            mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad <= th41
            && th41 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad
            &&
            mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.lowerAngleRad <= th51
            && th51 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.upperAngleRad
          )
        {
            aTh3 = th31;
            aTh4 = th41;
            aTh5 = th51;
        }
        else if( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad <= th32
                 && th32 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad
                 &&
                 mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad <= th42
                 && th42 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad
                 &&
                 mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.lowerAngleRad <= th52
                 && th52 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.upperAngleRad
               )
        {
            aTh3 = th32;
            aTh4 = th42;
            aTh5 = th52;
        }
        else
        {
            status = false;
        }
    }

    if( status )
    {
        status = checkJointAngles( aTh2, aTh3, aTh4, aTh5 );
    }

    return status;
}


//!************************************************************************
//! Compute the inverse kinematic model for 4DOF
//! ***  th5 = -th3  ***
//! It provides an exact analytical solution.
//! This function should be called from computeIkm()
//!
//! @returns true if the solution exists
//!************************************************************************
bool ADI_4DOF_KinematicsPlugin::computeIkmTh5EqMth3
    (
    const double aX,    //!< end-effector x coordinate
    const double aY,    //!< end-effector y coordinate
    const double aZ,    //!< end-effector z coordinate
    const double aL1,   //!< length of base link
    const double aL2,   //!< length of column link
    const double aL3,   //!< length of upper arm link
    const double aL4,   //!< length of lower arm link
    const double aL5,   //!< length of end effector link
    double&      aTh2,  //!< angle of base joint [rad]
    double&      aTh3,  //!< angle of shoulder joint [rad]
    double&      aTh4,  //!< angle of elbow joint [rad]
    double&      aTh5   //!< angle of wrist joint [rad]
    ) const
{
    bool status = true;

    aTh2 = atan2( aY, aX );
    refineTh2Angle( aTh2 );

    aTh3 = 0;
    aTh4 = 0;
    aTh5 = 0;

    double th31 = 0;
    double th32 = 0;

    double th41 = 0;
    double th42 = 0;

    double th51 = 0;
    double th52 = 0;

    double a = 0;
    double b = 0;

    double u1 = 0;
    double u2 = 0;
    double u3 = 0;
    double u4 = 0;
    double u5 = 0;

    double v1 = 0;
    double v2 = 0;
    double v3 = 0;
    double v4 = 0;
    double v5 = 0;
    double v6 = 0;
    double v7 = 0;
    double v8 = 0;
    double v9 = 0;
    double v10 = 0;
    double v11 = 0;
    double v12 = 0;
    double v13 = 0;
    double v14 = 0;
    double v15 = 0;
    double v16 = 0;
    double v17 = 0;
    double v18 = 0;
    double v19 = 0;
    double v20 = 0;
    double v21 = 0;
    double v22 = 0;

    if( 0 == sin( aTh2 ) )
    {
        a = aX / cos( aTh2 );
    }
    else
    {
        a = aY / sin( aTh2 );
    }

    b = aZ - aL1 - aL2;

    // th4{1,2}
    u1 = a * a + b * b - aL3 * aL3 - aL4 * aL4 + aL5 * aL5 + 2 * ( a * aL5 + aL3 * aL4 );
    u2 = 2 * ( aL3 * aL3 + aL4 * aL4 + aL5 * aL5 );
    u3 = a * a * ( -a * a - 2 * b * b + u2 ) + b * b * ( -b * b + u2 ) + 8 * a * aL3 * aL4 * aL5
            - pow( aL3, 4.0 ) - pow( aL4, 4.0 ) - pow( aL5, 4.0 )
            + 2 * ( pow( aL3 * aL4, 2.0 ) + pow( aL3 * aL5, 2.0 ) + pow( aL4 * aL5, 2.0 ) );

    if( u3 >= -ZERO_L_THD && u3 < 0 )
    {
        u3 = 0;
    }

    if( u3 < 0 )
    {
        status = false;
    }

    if( status )
    {
        u4 = sqrt( u3 );
        u5 = 2 * b * aL5;

        th41 = 2 * atan2( u5 - u4, u1 );
        th41 = 2 * atan2( u5 + u4, u1 );

        refineAnglePlusMinusPi( th41 );
        refineAnglePlusMinusPi( th42 );
    }

    // th3{1,2}
    if( status )
    {
        v1 = a * a + b * b - aL3 * aL3 - aL4 * aL4 + aL5 * aL5 + 2 * ( a * aL5 + aL3 * aL4 );

        if( 0 == v1 )
        {
            status = false;
        }
    }

    if( status )
    {
        v2 = a * a + b * b + aL3 * aL3 - aL4 * aL4 - aL5 * aL5 + 2 * ( a * aL3 + aL4 * aL5 );
        v3 = u3;
        v4 = sqrt( v3 );
        v5 = u5;
        v6 = v5 - v4;
        v7 = v5 + v4;

        double v61 = v6 / v1;
        double v71 = v7 / v1;

        v8 = a * a * v71;
        v9 = b * b * v71;
        v10 = aL3 * aL3 * v71;
        v11 = aL4 * aL4 * v71;
        v12 = aL5 * aL5 * v71;
        v13 = 2 * a * aL5 * v71;
        v14 = 2 * aL3 * aL4 * v71;

        v15 = a * a * v61;
        v16 = b * b * v61;
        v17 = aL3 * aL3 * v61;
        v18 = aL4 * aL4 * v61;
        v19 = aL5 * aL5 * v61;
        v20 = 2 * a * aL5 * v61;
        v21 = 2 * aL3 * aL4 * v61;

        v22 = 2 * b * ( aL3 + aL5 );

        th31 = -2 * atan2( v15 + v16 - v17 - v18 + v19 + v20 + v21 - v22, v2 );
        th32 = -2 * atan2( v8 + v9 - v10 - v11 + v12 + v13 + v14 - v22, v2 );

        refineAnglePlusMinusPi( th31 );
        refineAnglePlusMinusPi( th32 );

        th51 = -th31;
        th52 = -th32;

        if( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad <= th31
            && th31 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad
            &&
            mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad <= th41
            && th41 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad
            &&
            mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.lowerAngleRad <= th51
            && th51 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.upperAngleRad
          )
        {
            aTh3 = th31;
            aTh4 = th41;
            aTh5 = th51;
        }
        else if( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad <= th32
                 && th32 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad
                 &&
                 mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad <= th42
                 && th42 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad
                 &&
                 mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.lowerAngleRad <= th52
                 && th52 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.upperAngleRad
               )
        {
            aTh3 = th32;
            aTh4 = th42;
            aTh5 = th52;
        }
        else
        {
            status = false;
        }
    }

    if( status )
    {
        status = checkJointAngles( aTh2, aTh3, aTh4, aTh5 );
    }

    return status;
}


//!************************************************************************
//! Compute the inverse kinematic model for 4DOF
//! ***  th4 = -th3  ***
//! It provides an exact analytical solution.
//! This function should be called from computeIkm()
//!
//! @returns true if the solution exists
//!************************************************************************
bool ADI_4DOF_KinematicsPlugin::computeIkmTh4EqMth3
    (
    const double aX,    //!< end-effector x coordinate
    const double aY,    //!< end-effector y coordinate
    const double aZ,    //!< end-effector z coordinate
    const double aL1,   //!< length of base link
    const double aL2,   //!< length of column link
    const double aL3,   //!< length of upper arm link
    const double aL4,   //!< length of lower arm link
    const double aL5,   //!< length of end effector link
    double&      aTh2,  //!< angle of base joint [rad]
    double&      aTh3,  //!< angle of shoulder joint [rad]
    double&      aTh4,  //!< angle of elbow joint [rad]
    double&      aTh5   //!< angle of wrist joint [rad]
    ) const
{
    bool status = true;

    aTh2 = atan2( aY, aX );
    refineTh2Angle( aTh2 );

    aTh3 = 0;
    aTh4 = 0;
    aTh5 = 0;

    double th31 = 0;
    double th32 = 0;

    double th41 = 0;
    double th42 = 0;

    double th51 = 0;
    double th52 = 0;

    double a = 0;
    double b = 0;

    double u1 = 0;
    double u2 = 0;
    double u3 = 0;
    double u4 = 0;
    double u5 = 0;
    double u6 = 0;

    double v1 = 0;
    double v2 = 0;
    double v3 = 0;
    double v4 = 0;
    double v5 = 0;
    double v6 = 0;
    double v7 = 0;
    double v8 = 0;
    double v9 = 0;
    double v10 = 0;
    double v11 = 0;
    double v12 = 0;
    double v13 = 0;
    double v14 = 0;
    double v15 = 0;
    double v16 = 0;
    double v17 = 0;
    double v18 = 0;
    double v19 = 0;
    double v20 = 0;
    double v21 = 0;
    double v22 = 0;

    if( 0 == sin( aTh2 ) )
    {
        a = aX / cos( aTh2 ) - aL4;
    }
    else
    {
        a = aY / sin( aTh2 ) - aL4;
    }

    b = aZ - aL1 - aL2;

    // th5{1,2}
    u1 = a * a + b * b - aL3 * aL3 - aL5 * aL5;
    u2 = 2 * aL3 * aL5;
    u3 = ( u1 + u2 ) * ( -u1 + u2 );

    if( u3 >= -ZERO_L_THD && u3 < 0 )
    {
        u3 = 0;
    }

    if( u3 < 0 )
    {
        status = false;
    }

    if( status )
    {
        u4 = sqrt( u3 );
        u5 = 2 * b * aL5;
        u6 = a * a + b * b - aL3 * aL3 + aL5 * aL5 + 2 * a * aL5;

        th51 = 2 * atan2( u5 + u4, u6 );
        th51 = 2 * atan2( u5 - u4, u6 );

        refineAnglePlusMinusPi( th51 );
        refineAnglePlusMinusPi( th52 );
    }

    // th3{1,2}
    if( status )
    {
        v1 = a * a + b * b + 2 * a * aL5;
        v2 = aL3 * aL3 - aL5 * aL5;
        v3 = v1 - v2;

        if( 0 == v3 )
        {
            status = false;
        }
    }

    if( status )
    {
        v4 = v1 + v2;
        v5 = u3;
        v6 = sqrt( v5 );
        v7 = u5;
        v8 = v7 - v6;
        v9 = v7 + v6;

        double v83 = v8 / v3;
        double v93 = v9 / v3;

        v10 = a * a * v83;
        v11 = b * b * v83;
        v12 = aL3 * aL3 * v83;
        v13 = aL5 * aL5 * v83;
        v14 = 2 * a * aL5 * v83;

        v15 = a * a * v93;
        v16 = b * b * v93;
        v17 = aL3 * aL3 * v93;
        v18 = aL5 * aL5 * v93;
        v19 = 2 * a * aL5 * v93;

        v20 = 2 * b * ( aL3 + aL5 );

        th31 = -2 * atan2( v15 + v16 - v17 + v18 + v19 - v20, v4 );
        th32 = -2 * atan2( v10 + v11 - v12 + v13 + v14 - v20, v4 );

        refineAnglePlusMinusPi( th31 );
        refineAnglePlusMinusPi( th32 );

        th41 = -th31;
        th42 = -th32;

        if( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad <= th31
            && th31 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad
            &&
            mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad <= th41
            && th41 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad
            &&
            mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.lowerAngleRad <= th51
            && th51 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.upperAngleRad
          )
        {
            aTh3 = th31;
            aTh4 = th41;
            aTh5 = th51;
        }
        else if( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad <= th32
                 && th32 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad
                 &&
                 mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad <= th42
                 && th42 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad
                 &&
                 mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.lowerAngleRad <= th52
                 && th52 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.upperAngleRad
               )
        {
            aTh3 = th32;
            aTh4 = th42;
            aTh5 = th52;
        }
        else
        {
            status = false;
        }
    }

    if( status )
    {
        status = checkJointAngles( aTh2, aTh3, aTh4, aTh5 );
    }

    return status;
}


//!************************************************************************
//! Compute the inverse kinematic model for 4DOF
//! ***  th5+th4 = PI/2  ***
//! It provides an exact analytical solution.
//! This function should be called from computeIkm()
//!
//! @returns true if the solution exists
//!************************************************************************
bool ADI_4DOF_KinematicsPlugin::computeIkmTh5Th4EqPi2
    (
    const double aX,    //!< end-effector x coordinate
    const double aY,    //!< end-effector y coordinate
    const double aZ,    //!< end-effector z coordinate
    const double aL1,   //!< length of base link
    const double aL2,   //!< length of column link
    const double aL3,   //!< length of upper arm link
    const double aL4,   //!< length of lower arm link
    const double aL5,   //!< length of end effector link
    double&      aTh2,  //!< angle of base joint [rad]
    double&      aTh3,  //!< angle of shoulder joint [rad]
    double&      aTh4,  //!< angle of elbow joint [rad]
    double&      aTh5   //!< angle of wrist joint [rad]
    ) const
{
    bool status = true;

    aTh2 = atan2( aY, aX );
    refineTh2Angle( aTh2 );

    aTh3 = 0;
    aTh4 = 0;
    aTh5 = 0;

    double th31 = 0;
    double th32 = 0;

    double th41 = 0;
    double th42 = 0;

    double th51 = 0;
    double th52 = 0;

    double a = 0;
    double b = 0;

    double u1 = 0;
    double u2 = 0;
    double u3 = 0;
    double u4 = 0;
    double u5 = 0;
    double u6 = 0;
    double u7 = 0;
    double u8 = 0;

    double v1 = 0;
    double v2 = 0;
    double v3 = 0;
    double v4 = 0;
    double v5 = 0;
    double v6 = 0;
    double v7 = 0;
    double v8 = 0;
    double v9 = 0;
    double v10 = 0;
    double v11 = 0;
    double v12 = 0;
    double v13 = 0;
    double v14 = 0;
    double v15 = 0;
    double v16 = 0;
    double v17 = 0;
    double v18 = 0;

    if( 0 == sin( aTh2 ) )
    {
        a = aX / cos( aTh2 );
    }
    else
    {
        a = aY / sin( aTh2 );
    }

    b = aZ - aL1 - aL2;

    // th4{1,2}
    u1 = a * a + b * b - aL3 * aL3 - aL4 * aL4 - aL5 * aL5 + 2 * aL3 * aL4;
    u2 = aL3 * aL3 + aL4 * aL4 + aL5 * aL5;
    u3 = aL3 * aL3 - 2 * ( aL4 * aL4 - aL5 * aL5 );
    u4 = aL4 * aL4 - aL5 * aL5;
    u5 = a * a * ( 2 * u2 - a * a - 2 * b * b ) + b * b * ( 2 * u2 - b * b ) - aL3 * aL3 * u3 - u4 * u4;

    if( u5 >= -ZERO_L_THD && u5 < 0 )
    {
        u5 = 0;
    }

    if( u5 < 0 )
    {
        status = false;
    }

    if( status )
    {
        u6 = sqrt( u5 );
        u7 = 2 * aL4 * aL5 - u6;
        u8 = 2 * aL4 * aL5 + u6;

        th41 = 2 * atan2( u7, u1 );
        th42 = 2 * atan2( u8, u1 );

        refineAnglePlusMinusPi( th41 );
        refineAnglePlusMinusPi( th42 );

        th51 = 0.5 * SerialRobot::PI - th41;
        th52 = 0.5 * SerialRobot::PI - th42;

        refineAnglePlusMinusPi( th51 );
        refineAnglePlusMinusPi( th52 );
    }

    // th3{1,2}
    if( status )
    {
        v1 = u6;
        v2 = u7;
        v3 = u8;
        v4 = a * a + b * b - aL3 * aL3 - aL4 * aL4 - aL5 * aL5 + 2 * aL3 * aL4;
        v5 = a * a + b * b + aL3 * aL3 - aL4 * aL4 + aL5 * aL5 + 2 * ( a * aL3 + b * aL5 );

        if( 0 == v4 )
        {
            status = false;
        }
    }

    if( status )
    {
        double v34 = v3 / v4;
        double v24 = v2 / v4;

        v6 = aL3 * aL3 * v34;
        v7 = aL4 * aL4 * v34;
        v8 = aL5 * aL5 * v34;
        v9 = a * a * v34;
        v10 = b * b * v34;
        v11 = 2 * aL3 * aL4 * v34;

        v12 = aL3 * aL3 * v24;
        v13 = aL4 * aL4 * v24;
        v14 = aL5 * aL5 * v24;
        v15 = a * a * v24;
        v16 = b * b * v24;
        v17 = 2 * aL3 * aL4 * v24;

        v18 = 2 * ( b * aL3 - a * aL5 + aL4 * aL5 );

        th31 = 2 * atan2( v18 + v12 + v13 + v14 - v15 - v16 - v17, v5 );
        th32 = 2 * atan2( v18 + v6 + v7 + v8 - v9 - v10 - v11, v5 );

        refineAnglePlusMinusPi( th31 );
        refineAnglePlusMinusPi( th32 );

        if( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad <= th31
            && th31 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad
            &&
            mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad <= th41
            && th41 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad
            &&
            mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.lowerAngleRad <= th51
            && th51 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.upperAngleRad
          )
        {
            aTh3 = th31;
            aTh4 = th41;
            aTh5 = th51;
        }
        else if( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad <= th32
                 && th32 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad
                 &&
                 mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad <= th42
                 && th42 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad
                 &&
                 mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.lowerAngleRad <= th52
                 && th52 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.upperAngleRad
               )
        {
            aTh3 = th32;
            aTh4 = th42;
            aTh5 = th52;
        }
        else
        {
            status = false;
        }
    }

    if( status )
    {
        status = checkJointAngles( aTh2, aTh3, aTh4, aTh5 );
    }

    return status;
}


//!************************************************************************
//! Compute the inverse kinematic model for 4DOF
//! ***  th5+th4 = -PI/2  ***
//! It provides an exact analytical solution.
//! This function should be called from computeIkm()
//!
//! @returns true if the solution exists
//!************************************************************************
bool ADI_4DOF_KinematicsPlugin::computeIkmTh5Th4EqMpi2
    (
    const double aX,    //!< end-effector x coordinate
    const double aY,    //!< end-effector y coordinate
    const double aZ,    //!< end-effector z coordinate
    const double aL1,   //!< length of base link
    const double aL2,   //!< length of column link
    const double aL3,   //!< length of upper arm link
    const double aL4,   //!< length of lower arm link
    const double aL5,   //!< length of end effector link
    double&      aTh2,  //!< angle of base joint [rad]
    double&      aTh3,  //!< angle of shoulder joint [rad]
    double&      aTh4,  //!< angle of elbow joint [rad]
    double&      aTh5   //!< angle of wrist joint [rad]
    ) const
{
    bool status = true;

    aTh2 = atan2( aY, aX );
    refineTh2Angle( aTh2 );

    aTh3 = 0;
    aTh4 = 0;
    aTh5 = 0;

    double th31 = 0;
    double th32 = 0;

    double th41 = 0;
    double th42 = 0;

    double th51 = 0;
    double th52 = 0;

    double a = 0;
    double b = 0;

    double u1 = 0;
    double u2 = 0;
    double u3 = 0;
    double u4 = 0;
    double u5 = 0;
    double u6 = 0;
    double u7 = 0;
    double u8 = 0;

    double v1 = 0;
    double v2 = 0;
    double v3 = 0;
    double v4 = 0;
    double v5 = 0;
    double v6 = 0;
    double v7 = 0;
    double v8 = 0;
    double v9 = 0;
    double v10 = 0;
    double v11 = 0;
    double v12 = 0;
    double v13 = 0;
    double v14 = 0;
    double v15 = 0;
    double v16 = 0;
    double v17 = 0;
    double v18 = 0;

    if( 0 == sin( aTh2 ) )
    {
        a = aX / cos( aTh2 );
    }
    else
    {
        a = aY / sin( aTh2 );
    }

    b = aZ - aL1 - aL2;

    // th4{1,2}
    u1 = a * a + b * b - aL3 * aL3 - aL4 * aL4 - aL5 * aL5 + 2 * aL3 * aL4;
    u2 = aL3 * aL3 + aL4 * aL4 + aL5 * aL5;
    u3 = aL3 * aL3 - 2 * ( aL4 * aL4 - aL5 * aL5 );
    u4 = aL4 * aL4 - aL5 * aL5;
    u5 = a * a * ( 2 * u2 - a * a - 2 * b * b ) + b * b * ( 2 * u2 - b * b ) - aL3 * aL3 * u3 - u4 * u4;

    if( u5 >= -ZERO_L_THD && u5 < 0 )
    {
        u5 = 0;
    }

    if( u5 < 0 )
    {
        status = false;
    }

    if( status )
    {
        u6 = sqrt( u5 );
        u7 = 2 * aL4 * aL5 - u6;
        u8 = 2 * aL4 * aL5 + u6;

        th41 = -2 * atan2( u7, u1 );
        th42 = -2 * atan2( u8, u1 );

        refineAnglePlusMinusPi( th41 );
        refineAnglePlusMinusPi( th42 );

        th51 = -0.5 * SerialRobot::PI - th41;
        th52 = -0.5 * SerialRobot::PI - th42;

        refineAnglePlusMinusPi( th51 );
        refineAnglePlusMinusPi( th52 );
    }

    // th3{1,2}
    if( status )
    {
        v1 = u6;
        v2 = u7;
        v3 = u8;
        v4 = a * a + b * b - aL3 * aL3 - aL4 * aL4 - aL5 * aL5 + 2 * aL3 * aL4;
        v5 = a * a + b * b + aL3 * aL3 - aL4 * aL4 + aL5 * aL5 + 2 * ( a * aL3 - b * aL5 );

        if( 0 == v4 )
        {
            status = false;
        }
    }

    if( status )
    {
        double v34 = v3 / v4;
        double v24 = v2 / v4;

        v6 = aL3 * aL3 * v34;
        v7 = aL4 * aL4 * v34;
        v8 = aL5 * aL5 * v34;
        v9 = a * a * v34;
        v10 = b * b * v34;
        v11 = 2 * aL3 * aL4 * v34;

        v12 = aL3 * aL3 * v24;
        v13 = aL4 * aL4 * v24;
        v14 = aL5 * aL5 * v24;
        v15 = a * a * v24;
        v16 = b * b * v24;
        v17 = 2 * aL3 * aL4 * v24;

        v18 = 2 * ( b * aL3 + a * aL5 - aL4 * aL5 );

        th31 = 2 * atan2( v18 - ( v12 + v13 + v14 - v15 - v16 - v17 ), v5 );
        th32 = 2 * atan2( v18 - ( v6 + v7 + v8 - v9 - v10 - v11 ), v5 );

        refineAnglePlusMinusPi( th31 );
        refineAnglePlusMinusPi( th32 );

        if( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad <= th31
            && th31 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad
            &&
            mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad <= th41
            && th41 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad
            &&
            mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.lowerAngleRad <= th51
            && th51 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.upperAngleRad
          )
        {
            aTh3 = th31;
            aTh4 = th41;
            aTh5 = th51;
        }
        else if( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad <= th32
                 && th32 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad
                 &&
                 mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad <= th42
                 && th42 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad
                 &&
                 mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.lowerAngleRad <= th52
                 && th52 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.upperAngleRad
               )
        {
            aTh3 = th32;
            aTh4 = th42;
            aTh5 = th52;
        }
        else
        {
            status = false;
        }
    }

    if( status )
    {
        status = checkJointAngles( aTh2, aTh3, aTh4, aTh5 );
    }

    return status;
}


//!************************************************************************
//! Compute the inverse kinematic model for 4DOF
//! ***  th5+th4 = PI  ***
//!  OR
//! ***  th5+th4 = -PI  ***
//! It provides an exact analytical solution.
//! This function should be called from computeIkm()
//!
//! @returns true if the solution exists
//!************************************************************************
bool ADI_4DOF_KinematicsPlugin::computeIkmTh5Th4EqPi
    (
    const double aX,    //!< end-effector x coordinate
    const double aY,    //!< end-effector y coordinate
    const double aZ,    //!< end-effector z coordinate
    const double aL1,   //!< length of base link
    const double aL2,   //!< length of column link
    const double aL3,   //!< length of upper arm link
    const double aL4,   //!< length of lower arm link
    const double aL5,   //!< length of end effector link
    double&      aTh2,  //!< angle of base joint [rad]
    double&      aTh3,  //!< angle of shoulder joint [rad]
    double&      aTh4,  //!< angle of elbow joint [rad]
    double&      aTh5   //!< angle of wrist joint [rad]
    ) const
{
    bool status = true;

    aTh2 = atan2( aY, aX );
    refineTh2Angle( aTh2 );

    aTh3 = 0;
    aTh4 = 0;
    aTh5 = 0;

    double th31 = 0;
    double th32 = 0;

    double th41 = 0;
    double th42 = 0;

    double th51 = 0;
    double th52 = 0;

    double a = 0;
    double b = 0;

    double u1 = 0;
    double u2 = 0;
    double u3 = 0;

    double v1 = 0;
    double v2 = 0;
    double v3 = 0;
    double v4 = 0;
    double v5 = 0;
    double v6 = 0;
    double v7 = 0;
    double v8 = 0;
    double v9 = 0;
    double v10 = 0;
    double v11 = 0;
    double v12 = 0;
    double v13 = 0;
    double v14 = 0;
    double v15 = 0;

    if( 0 == sin( aTh2 ) )
    {
        a = aX / cos( aTh2 );
    }
    else
    {
        a = aY / sin( aTh2 );
    }

    b = aZ - aL1 - aL2;

    // th4{1,2}
    u1 = a * a + b * b - aL3 * aL3 - aL4 * aL4 - aL5 * aL5 + 2 * aL3 * aL5;
    u2 = 2 * aL4 * ( aL3 - aL5 );
    u3 = ( u1 + u2 ) * ( -u1 + u2 );

    if( u3 >= -ZERO_L_THD && u3 < 0 )
    {
        u3 = 0;
    }

    if( u3 < 0 )
    {
        status = false;
    }

    if( status )
    {
        th41 = 2 * atan2( sqrt( u3 ), u1 + u2 );
        th42 = -th41;

        refineAnglePlusMinusPi( th41 );
        refineAnglePlusMinusPi( th42 );
    }

    // th3{1,2}
    if( status )
    {
        v1 = a * a + b * b + aL3 * aL3 - aL4 * aL4 + aL5 * aL5 - 2 * aL3 * aL5 + 2 * a * ( aL3 - aL5 );
        v2 = a * a + b * b - aL3 * aL3 - aL4 * aL4 - aL5 * aL5 + 2 * aL3 * aL5;
        v3 = 2 * aL4 * ( aL3 - aL5 );
        v4 = v2 + v3;

        if( 0 == v4 )
        {
            status = false;
        }
    }

    double vprod = v4 * ( -v2 + v3 );

    if( vprod >= -ZERO_L_THD && vprod < 0 )
    {
        vprod = 0;
    }

    if( status )
    {
        if( vprod < 0 )
        {
            status = false;
        }
    }

    if( status )
    {
        v5 = sqrt( vprod );
        double v54 = v5 / v4;
        v6 = aL5 * aL5 * v54;
        v7 = aL4 * aL4 * v54;
        v8 = aL3 * aL3 * v54;
        v9 = b * b * v54;
        v10 = a * a * v54;
        v11 = 2 * aL4 * aL5 * v54;
        v12 = 2 * aL3 * aL5 * v54;
        v13 = 2 * aL3 * aL4 * v54;
        v14 = v10 + v9 - v8 - v7 - v6 + v13 + v12 - v11;
        v15 = 2 * b * ( aL3 - aL5 );

        th31 = 2 * atan2( v15 - v14, v1 );
        th32 = 2 * atan2( v15 + v14, v1 );

        refineAnglePlusMinusPi( th31 );
        refineAnglePlusMinusPi( th32 );

        bool casePlusPi = false;
        bool caseMinusPi = false;

        if( !casePlusPi )
        {
            th51 = SerialRobot::PI - th41;
            th52 = SerialRobot::PI - th42;

            refineAnglePlusMinusPi( th51 );
            refineAnglePlusMinusPi( th52 );

            refineAnglesPlusPi( th41, th51 );
            refineAnglesPlusPi( th42, th52 );

            if( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad <= th31
                && th31 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad
                &&
                mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad <= th41
                && th41 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad
                &&
                mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.lowerAngleRad <= th51
                && th51 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.upperAngleRad
              )
            {
                aTh3 = th31;
                aTh4 = th41;
                aTh5 = th51;

                casePlusPi = true;
            }
            else if( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad <= th32
                     && th32 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad
                     &&
                     mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad <= th42
                     && th42 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad
                     &&
                     mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.lowerAngleRad <= th52
                     && th52 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.upperAngleRad
                   )
            {
                aTh3 = th32;
                aTh4 = th42;
                aTh5 = th52;

                casePlusPi = true;
            }
        }

        if( !casePlusPi )
        {
            // case II
            th51 = -SerialRobot::PI - th41;
            th52 = -SerialRobot::PI - th42;

            refineAnglePlusMinusPi( th51 );
            refineAnglePlusMinusPi( th52 );

            refineAnglesMinusPi( th41, th51 );
            refineAnglesMinusPi( th42, th52 );

            if( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad <= th31
                && th31 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad
                &&
                mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad <= th41
                && th41 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad
                &&
                mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.lowerAngleRad <= th51
                && th51 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.upperAngleRad
              )
            {
                aTh3 = th31;
                aTh4 = th41;
                aTh5 = th51;

                caseMinusPi = true;
            }
            else if( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad <= th32
                     && th32 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad
                     &&
                     mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad <= th42
                     && th42 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad
                     &&
                     mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.lowerAngleRad <= th52
                     && th52 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.upperAngleRad
                   )
            {
                aTh3 = th32;
                aTh4 = th42;
                aTh5 = th52;

                caseMinusPi = true;
            }
        }

        if( !casePlusPi && !caseMinusPi )
        {
            status = false;
        }
    }

    if( status )
    {
        status = checkJointAngles( aTh2, aTh3, aTh4, aTh5 );
    }

    return status;
}


//!************************************************************************
//! Calculate the rank for a set of new joint angles, considering the
//! current values of the joint angles.
//! The cost hierarchy is:
//!     base (max. cost) -> shoulder -> elbow -> wrist (min. cost)
//!
//! => rank=0 means maximum total cost (full movement)
//! => rank=1 means minimum total cost (no movement)
//!
//! @returns the rank of the new joint angles
//!************************************************************************
double ADI_4DOF_KinematicsPlugin::computeJointAnglesRank
    (
    const double aTh2,  //!< angle of base joint [rad]
    const double aTh3,  //!< angle of shoulder joint [rad]
    const double aTh4,  //!< angle of elbow joint [rad]
    const double aTh5   //!< angle of wrist joint [rad]
    ) const
{
    double deltaMaxTh2 = mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_JOINT ).joint.limit.upperAngleRad -
                         mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_JOINT ).joint.limit.lowerAngleRad;
    double deltaMaxTh3 = mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad -
                         mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad;
    double deltaMaxTh4 = mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad -
                         mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad;
    double deltaMaxTh5 = mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.upperAngleRad -
                         mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.lowerAngleRad;

    double diffTh2 = fabs( aTh2 - gJointAngles.th2 );
    double diffTh3 = fabs( aTh3 - gJointAngles.th3 );
    double diffTh4 = fabs( aTh4 - gJointAngles.th4 );
    double diffTh5 = fabs( aTh5 - gJointAngles.th5 );

    double costTh2 = ( 8.0 / 15.0 ) * ( diffTh2 / deltaMaxTh2 );
    double costTh3 = ( 4.0 / 15.0 ) * ( diffTh3 / deltaMaxTh3 );
    double costTh4 = ( 2.0 / 15.0 ) * ( diffTh4 / deltaMaxTh4 );
    double costTh5 = ( 1.0 / 15.0 ) * ( diffTh5 / deltaMaxTh5 );

    double rank = 1.0 - ( costTh2 + costTh3 + costTh4 + costTh5 );
    return rank;
}


//!************************************************************************
//! Convert an angle from degrees to radians
//!
//! @returns the value in radians
//!************************************************************************
double ADI_4DOF_KinematicsPlugin::convertDegToRad
    (
    const double aValue //!< value
    ) const
{
    return aValue * SerialRobot::PI / 180.0;
}


//!************************************************************************
//! Convert an angle from radians to degrees
//!
//! @returns the value in degrees
//!************************************************************************
double ADI_4DOF_KinematicsPlugin::convertRadToDeg
    (
    const double aValue //!< value
    ) const
{
    return aValue * 180.0 / SerialRobot::PI;
}


//!************************************************************************
//! Finds the index in a joint angles vector whose rank is maximum
//! See description in computeJointAnglesRank().
//!
//! @returns the index in vector where rank is maximum
//!************************************************************************
size_t ADI_4DOF_KinematicsPlugin::findBestRank
    (
    const std::vector<JointAngles>& aJointAnglesVec //!< joint angles vector
    ) const
{
    size_t index = 0;
    size_t vecSize = aJointAnglesVec.size();

    if( vecSize )
    {
        std::vector<double> rankVec;

        for( size_t i = 0; i < vecSize; i++ )
        {
            rankVec.push_back( aJointAnglesVec.at( i ).rank );
        }

        index = std::distance( rankVec.begin(), std::max_element( rankVec.begin(), rankVec.end() ) );
    }

    return index;
}


//!************************************************************************
//! Get the joint names
//!
//! @returns the strings vector with joint names
//!************************************************************************
const std::vector<std::string>& ADI_4DOF_KinematicsPlugin::getJointNames() const
{
    return mJointNames;
}


//!************************************************************************
//! Get the KDL segment index
//!
//! @returns the segment index
//!************************************************************************
int ADI_4DOF_KinematicsPlugin::getKDLSegmentIndex
    (
    const std::string&  aName   //!< name
    ) const
{  
    int index = -1;

    for( int i = 0; i < static_cast<int>( mChain.getNrOfSegments() ); i++ )
    {
        if( aName == mChain.getSegment( i ).getName() )
        {
            index = i + 1;
            break;
        }
    }

    return index;
}


//!************************************************************************
//! Get the link names
//!
//! @returns the strings vector with link names
//!************************************************************************
const std::vector<std::string>& ADI_4DOF_KinematicsPlugin::getLinkNames() const
{
    return mLinkNames;
}


//!************************************************************************
//! Forward kinematics computations.
//! It is only used if 'use_plugin_fk' is set in the
//! 'arm_kinematics_constraint_aware' node, otherwise ROS TF is used to
//! calculate the forward kinematics.
//!
//! @returns true if the forward kinematics can be calculated
//!************************************************************************
bool ADI_4DOF_KinematicsPlugin::getPositionFK
    (
    const std::vector<std::string>&         aLinkNames,     //!< set of links for which FK needs to be computed
    const std::vector<double>&              aJointAngles,   //!< joint angles for which FK needs to be computed
    std::vector<geometry_msgs::msg::Pose>&  aPoses          //!< the resultant set of poses
    ) const
{
    bool status = false;

    if( mIsActive )
    {
        aPoses.resize( aLinkNames.size() );

        if( aJointAngles.size() == mNumJoints )
        {
            KDL::JntArray jointPosIn( mNumJoints );

            for( uint8_t i = 0; i < mNumJoints; i++ )
            {
                jointPosIn( i ) = aJointAngles[i];
            }

            KDL::Frame pOut;
            KDL::ChainFkSolverPos_recursive fkSolver( mChain );
            status = true;

            for( uint8_t i = 0; i < aPoses.size(); i++ )
            {
                if( fkSolver.JntToCart( jointPosIn, pOut, getKDLSegmentIndex( aLinkNames[i] ) ) >= 0 )
                {
                    aPoses[i] = tf2::toMsg( pOut );
                }
                else
                {
                    RCLCPP_ERROR( LOGGER, "Could not compute FK for %s", aLinkNames[i].c_str() );
                    status = false;
                }
            }
        }
        else
        {
            RCLCPP_ERROR( LOGGER, "Joint vector size is %ld, expected %d", aJointAngles.size(), mNumJoints );
        }
    }
    else
    {
        RCLCPP_ERROR( LOGGER, "Kinematics is not active" );
    }

    return status;
}


//!************************************************************************
//! Given a desired pose of the end-effector, compute the joint angles to
//! reach it.
//!
//! @returns true if an inverse kinematics solution is found
//!************************************************************************
bool ADI_4DOF_KinematicsPlugin::getPositionIK
    (
    const geometry_msgs::msg::Pose&            aIkPose,        //!< target end-effector pose
    const std::vector<double>&                 aIkSeedState,   //!< seed (not used)
    std::vector<double>&                       aSolution,      //!< solution vector
    moveit_msgs::msg::MoveItErrorCodes&        aErrorCode,     //!< error code
    const kinematics::KinematicsQueryOptions&  aOptions        //!< options (not used)
    ) const
{
    double defaultTimeout = 0;
    const IKCallbackFn solutionCallback = 0;
    std::vector<double> consistencyLimits;

    return searchPositionIK( aIkPose,
                             aIkSeedState,
                             defaultTimeout,
                             aSolution,
                             solutionCallback,
                             aErrorCode,
                             consistencyLimits,
                             aOptions );
}


//!************************************************************************
//! Kinematics initialization
//! Fast init, because it uses a previously parsed URDF model.
//! Read current parameters of links and joints which are relevant.
//!
//! @returns true if initialization can be done
//!************************************************************************
bool ADI_4DOF_KinematicsPlugin::initialize
    (
    const rclcpp::Node::SharedPtr&  aNode,                  //!< node
    const moveit::core::RobotModel& aRobotModel,            //!< robot model
    const std::string&              aGroupName,             //!< group name
    const std::string&              aBaseFrame,             //!< base frame
    const std::vector<std::string>& aTipFrames,             //!< tip frames
    double                          aSearchDiscretization   //!< search discretization
    )
{
    bool status = true;
    node_ = aNode;
    storeValues( aRobotModel, aGroupName, aBaseFrame, aTipFrames, aSearchDiscretization );
    KDL::Tree kdlTree;

    if( status )
    {
        status = kdl_parser::treeFromUrdfModel( *aRobotModel.getURDF(), kdlTree );

        if( !status )
        {
            RCLCPP_FATAL( LOGGER, "Failed to extract KDL tree from XML robot description" );
        }
    }

    if( status )
    {
        size_t tipFramesSize = aTipFrames.size();
        status = ( 1 == tipFramesSize );

        if( !status )
        {
            RCLCPP_FATAL( LOGGER, "Tip frames size found %lu instead of 1", tipFramesSize );
        }
    }

    if( status )
    {
        status = kdlTree.getChain( aBaseFrame, aTipFrames[0], mChain );

        if( !status )
        {
            RCLCPP_FATAL( LOGGER, "Couldn't find chain %s to %s", aBaseFrame.c_str(), aTipFrames[0].c_str() );
        }
    }

    if( status )
    {
        mNumJoints = mChain.getNrOfJoints();

        if( NR_JOINTS != mNumJoints )
        {
            status = false;
            RCLCPP_FATAL( LOGGER, "Expected 4 joints in the robotarm, there are %d.", mNumJoints );
        }
    }

    if( status )
    {
        std::vector<KDL::Segment> chainSegmentsVec = mChain.segments;
        uint8_t jointNr = 0;

        for( size_t i = 0; i < chainSegmentsVec.size(); i++ )
        {
            //************************
            // links
            //************************
            std::string crtLinkName = chainSegmentsVec.at( i ).getName();
            mLinkNames.push_back( crtLinkName );

            double crtLinkLength = 0;
            urdf::LinkConstSharedPtr link = robot_model_->getURDF()->getLink( crtLinkName );

            if( urdf::Geometry::CYLINDER == link->visual->geometry->type )
            {
                urdf::CylinderSharedPtr cylinder = std::dynamic_pointer_cast<urdf::Cylinder>( link->visual->geometry );
                crtLinkLength = cylinder->length; // meters
            }

            if( SerialRobot::PARTS_CHAIN_NAMES.at( SerialRobot::PARTS_CHAIN_BASE_LINK ) == crtLinkName )
            {
                mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_LINK ).link.geometry.cylinder.length = crtLinkLength;
            }
            else if( SerialRobot::PARTS_CHAIN_NAMES.at( SerialRobot::PARTS_CHAIN_COLUMN_LINK ) == crtLinkName )
            {
                mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_COLUMN_LINK ).link.geometry.cylinder.length = crtLinkLength;
            }
            else if( SerialRobot::PARTS_CHAIN_NAMES.at( SerialRobot::PARTS_CHAIN_UPPER_ARM_LINK ) == crtLinkName )
            {
                mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_UPPER_ARM_LINK ).link.geometry.cylinder.length = crtLinkLength;
            }
            else if( SerialRobot::PARTS_CHAIN_NAMES.at( SerialRobot::PARTS_CHAIN_LOWER_ARM_LINK ) == crtLinkName )
            {
                mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_LOWER_ARM_LINK ).link.geometry.cylinder.length = crtLinkLength;
            }
            else if( SerialRobot::PARTS_CHAIN_NAMES.at( SerialRobot::PARTS_CHAIN_END_EFFECTOR_LINK ) == crtLinkName )
            {
                mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_END_EFFECTOR_LINK ).link.geometry.cylinder.length = crtLinkLength;
            }

            //************************
            // joints
            //************************
            urdf::JointConstSharedPtr joint = robot_model_->getURDF()->getJoint( chainSegmentsVec.at( i ).getJoint().getName() );

            if( joint->type != urdf::Joint::UNKNOWN
             && joint->type != urdf::Joint::FIXED )
            {
                jointNr++;
                assert( jointNr <= mNumJoints );

                std::string crtJointName = joint->name;
                mJointNames.push_back( crtJointName );

                double lower = 0; // radians
                double upper = 0; // radians

                if( urdf::Joint::REVOLUTE == joint->type )
                {
                    if( joint->safety )
                    {
                        lower = std::max( joint->limits->lower, joint->safety->soft_lower_limit );
                        upper = std::min( joint->limits->upper, joint->safety->soft_upper_limit );
                    }
                    else
                    {
                        lower = joint->limits->lower;
                        upper = joint->limits->upper;
                    }
                }

                if( SerialRobot::PARTS_CHAIN_NAMES.at( SerialRobot::PARTS_CHAIN_BASE_JOINT ) == crtJointName )
                {
                    mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_JOINT ).joint.limit.lowerAngleRad = lower;
                    mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_JOINT ).joint.limit.upperAngleRad = upper;
                }
                else if( SerialRobot::PARTS_CHAIN_NAMES.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ) == crtJointName )
                {
                    mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad = lower;
                    mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad = upper;
                }
                else if( SerialRobot::PARTS_CHAIN_NAMES.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ) == crtJointName )
                {
                    mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad = lower;
                    mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad = upper;
                }
                else if( SerialRobot::PARTS_CHAIN_NAMES.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ) == crtJointName )
                {
                    mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.lowerAngleRad = lower;
                    mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.upperAngleRad = upper;
                }
            }
        }
    }

    mIsActive = status;

    if( !status )
    {
        RCLCPP_FATAL( LOGGER, "\n\n ADI_4DOF_KinematicsPlugin::initialize() FAILED.\n" );
    }

    return status;
}


//!************************************************************************
//! Refine an angle to belong in [-PI..PI]
//!
//! @returns nothing
//!************************************************************************
void ADI_4DOF_KinematicsPlugin::refineAnglePlusMinusPi
    (
    double& aValue      //!< angle [rad]
    ) const
{
    if( aValue < -SerialRobot::PI )
    {
        aValue += 2 * SerialRobot::PI;
    }
    else if( aValue > SerialRobot::PI )
    {
        aValue -= 2 * SerialRobot::PI;
    }
}


//!************************************************************************
//! Refine two angles which are both close to -PI/2.
//!
//! @returns nothing
//!************************************************************************
void ADI_4DOF_KinematicsPlugin::refineAnglesMinusPi
    (
    double& aTh4,       //!< angle of elbow joint [rad]
    double& aTh5        //!< angle of wrist joint [rad]
    ) const
{
    if( fabs( aTh4 + 0.5 * SerialRobot::PI ) <= ZERO_RAD_THD
     && fabs( aTh5 + 0.5 * SerialRobot::PI ) <= ZERO_RAD_THD
      )
    {
        aTh4 = -0.5 * SerialRobot::PI;
        aTh5 = -0.5 * SerialRobot::PI;
    }
}


//!************************************************************************
//! Refine two angles which are both close to +PI/2.
//!
//! @returns nothing
//!************************************************************************
void ADI_4DOF_KinematicsPlugin::refineAnglesPlusPi
    (
    double& aTh4,       //!< angle of elbow joint [rad]
    double& aTh5        //!< angle of wrist joint [rad]
    ) const
{
    if( fabs( aTh4 - 0.5 * SerialRobot::PI ) <= ZERO_RAD_THD
     && fabs( aTh5 - 0.5 * SerialRobot::PI ) <= ZERO_RAD_THD
      )
    {
        aTh4 = 0.5 * SerialRobot::PI;
        aTh5 = 0.5 * SerialRobot::PI;
    }
}


//!************************************************************************
//! Refine the angle of base joint, based on tan() periodicity and limits.
//! Note: th2=0 is between 3rd and 4th quadrants.
//!
//! @returns nothing
//!************************************************************************
void ADI_4DOF_KinematicsPlugin::refineTh2Angle
    (
    double& aTh2        //!< angle of base joint [rad]
    ) const
{
    double min = mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_JOINT ).joint.limit.lowerAngleRad;
    double max = mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_JOINT ).joint.limit.upperAngleRad;

    if( aTh2 > max
     && aTh2 - SerialRobot::PI >= min )
    {
        aTh2 -= SerialRobot::PI;
    }
    else if( aTh2 < min
          && aTh2 + SerialRobot::PI <= max )
    {
        aTh2 += SerialRobot::PI;
    }
}


//!************************************************************************
//! Given a desired pose of the end-effector, search for the joint angles
//! required to reach it. Intended for "searching" for solutions by
//! stepping trypically through the redundancy.
//!
//! @returns true if an inverse kinematics solution is found
//!************************************************************************
bool ADI_4DOF_KinematicsPlugin::searchPositionIK
    (
    const geometry_msgs::msg::Pose&             aIkPose,        //!< target end-effector pose
    const std::vector<double>&                  aIkSeedState,   //!< seed (not used)
    double                                      aTimeout,       //!< timeout (not used)
    std::vector<double>&                        aSolution,      //!< solution vector
    moveit_msgs::msg::MoveItErrorCodes&         aErrorCode,     //!< error code
    const kinematics::KinematicsQueryOptions&   aOptions        //!< options (not used)
    ) const
{
    const IKCallbackFn solutionCallback = 0;
    std::vector<double> consistencyLimits;

    return searchPositionIK( aIkPose,
                             aIkSeedState,
                             aTimeout,
                             aSolution,
                             solutionCallback,
                             aErrorCode,
                             consistencyLimits,
                             aOptions );
}


//!************************************************************************
//! Given a desired pose of the end-effector, search for the joint angles
//! required to reach it. Intended for "searching" for solutions by
//! stepping trypically through the redundancy.
//!
//! @returns true if an inverse kinematics solution is found
//!************************************************************************
bool ADI_4DOF_KinematicsPlugin::searchPositionIK
    (
    const geometry_msgs::msg::Pose&             aIkPose,            //!< target end-effector pose
    const std::vector<double>&                  aIkSeedState,       //!< seed (not used)
    double                                      aTimeout,           //!< timeout (not used)
    const std::vector<double>&                  aConsistencyLimits, //!< consistency limits (not used)
    std::vector<double>&                        aSolution,          //!< solution vector
    moveit_msgs::msg::MoveItErrorCodes&         aErrorCode,         //!< error code
    const kinematics::KinematicsQueryOptions&   aOptions            //!< options (not used)
    ) const
{
    const IKCallbackFn solutionCallback = 0;

    return searchPositionIK( aIkPose,
                             aIkSeedState,
                             aTimeout,
                             aSolution,
                             solutionCallback,
                             aErrorCode,
                             aConsistencyLimits,
                             aOptions );
}


//!************************************************************************
//! Given a desired pose of the end-effector, search for the joint angles
//! required to reach it. Intended for "searching" for solutions by
//! stepping trypically through the redundancy.
//!
//! @returns true if an inverse kinematics solution is found
//!************************************************************************
bool ADI_4DOF_KinematicsPlugin::searchPositionIK
    (
    const geometry_msgs::msg::Pose&             aIkPose,            //!< target end-effector pose
    const std::vector<double>&                  aIkSeedState,       //!< seed (not used)
    double                                      aTimeout,           //!< timeout (not used)
    std::vector<double>&                        aSolution,          //!< solution vector
    const IKCallbackFn&                         aSolutionCallback,  //!< callback function (not used)
    moveit_msgs::msg::MoveItErrorCodes&         aErrorCode,         //!< error code
    const kinematics::KinematicsQueryOptions&   aOptions            //!< options (not used)
    ) const
{
    std::vector<double> consistencyLimits;

    return searchPositionIK( aIkPose,
                             aIkSeedState,
                             aTimeout,
                             aSolution,
                             aSolutionCallback,
                             aErrorCode,
                             consistencyLimits,
                             aOptions );
}


//!************************************************************************
//! Given a desired pose of the end-effector, search for the joint angles
//! required to reach it. Intended for "searching" for solutions by
//! stepping trypically through the redundancy.
//!
//! @returns true if an inverse kinematics solution is found
//!************************************************************************
bool ADI_4DOF_KinematicsPlugin::searchPositionIK
    (
    const geometry_msgs::msg::Pose&             aIkPose,            //!< target end-effector pose
    const std::vector<double>&                  aIkSeedState,       //!< seed (not used)
    double                                      aTimeout,           //!< timeout (not used)
    const std::vector<double>&                  aConsistencyLimits, //!< consistency limits (not used)
    std::vector<double>&                        aSolution,          //!< solution vector
    const IKCallbackFn&                         aSolutionCallback,  //!< callback function (not used)
    moveit_msgs::msg::MoveItErrorCodes&         aErrorCode,         //!< error code
    const kinematics::KinematicsQueryOptions&   aOptions            //!< options (not used)
    ) const
{
    return searchPositionIK( aIkPose,
                             aIkSeedState,
                             aTimeout,
                             aSolution,
                             aSolutionCallback,
                             aErrorCode,
                             aConsistencyLimits,
                             aOptions );
}


//!************************************************************************
//! Given a desired pose of the end-effector, search for the joint angles
//! required to reach it.
//! Implementation for a 4DOF (4R) robot arm.
//!
//! @returns true if an inverse kinematics solution is found
//!************************************************************************
bool ADI_4DOF_KinematicsPlugin::searchPositionIK
    (
    const geometry_msgs::msg::Pose&             aIkPose,            //!< target end-effector pose
    const std::vector<double>&                  aIkSeedState,       //!< seed (not used)
    double                                      aTimeout,           //!< timeout (not used)
    std::vector<double>&                        aSolution,          //!< solution vector
    const IKCallbackFn&                         aSolutionCallback,  //!< callback function (not used)
    moveit_msgs::msg::MoveItErrorCodes&         aErrorCode,         //!< error code
    const std::vector<double>&                  aConsistencyLimits, //!< consistency limits (not used)
    const kinematics::KinematicsQueryOptions&   aOptions            //!< options (not used)
    ) const
{
    bool status = true;

    if( !mIsActive )
    {
        status = false;
        RCLCPP_ERROR( LOGGER, "Kinematics is not active." );
    }

    if( NR_JOINTS != mNumJoints )
    {
        status = false;
        RCLCPP_FATAL( LOGGER, "Expected 4 joints in the robotarm, there are %d.", mNumJoints );
    }

    if( status )
    {
        aSolution.clear();
        aSolution.resize( mNumJoints );

        KDL::Frame frame;
        tf2::fromMsg( aIkPose, frame );

        double x = frame.p.x();
        double y = frame.p.y();
        double z = frame.p.z();

        RCLCPP_INFO( LOGGER, " " );
        RCLCPP_INFO( LOGGER, " ADI 4DOF IK -> x=%lf, y=%lf, z=%lf", x, y, z );

        std::vector<JointAngles> jointAnglesVec;

        status = computeIkm( x, y, z,
                             mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_LINK ).link.geometry.cylinder.length,
                             mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_COLUMN_LINK ).link.geometry.cylinder.length,
                             mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_UPPER_ARM_LINK ).link.geometry.cylinder.length,
                             mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_LOWER_ARM_LINK ).link.geometry.cylinder.length,
                             mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_END_EFFECTOR_LINK ).link.geometry.cylinder.length,
                             jointAnglesVec );

        if( !status )
        {
            RCLCPP_INFO( LOGGER, " => no IK solution found." );
        }
        else
        {
            size_t solutionsCount = jointAnglesVec.size();

            if( !solutionsCount )
            {
                status = false;
                RCLCPP_INFO( LOGGER, " => no IK solution found." );
            }
            else
            {
                size_t brIndex = findBestRank( jointAnglesVec );
                aSolution.resize( mNumJoints );                
                bool updatePrevious = true;

                if( updatePrevious )
                {
                    updateJointAngles( jointAnglesVec.at( brIndex ) );
                }

                RCLCPP_INFO( LOGGER, " => found %lu solution%s", solutionsCount, solutionsCount > 1 ? "s" : "" );
                RCLCPP_INFO( LOGGER, " => Selected solution: th2=%lf, th3=%lf, th4=%lf, th5=%lf",
                             convertRadToDeg( jointAnglesVec.at( brIndex ).th2 ),
                             convertRadToDeg( jointAnglesVec.at( brIndex ).th3 ),
                             convertRadToDeg( jointAnglesVec.at( brIndex ).th4 ),
                             convertRadToDeg( jointAnglesVec.at( brIndex ).th5 ) );

                //**********************
                // base joint
                //**********************
                aSolution.at( 0 ) = jointAnglesVec.at( brIndex ).th2;

                //**********************
                // shoulder joint
                //
                //  MoveIt  |  IK model
                // ---------+----------
                //   -PI/2  |   n/a
                //     0    |   PI/2    (vertical relative to base plane)
                //    PI/2  |    0      (horizontal relative to base plane)
                //**********************
                aSolution.at( 1 ) = fabs( jointAnglesVec.at( brIndex ).th3 - 0.5 * SerialRobot::PI );

                //**********************
                // elbow joint
                //
                //  MoveIt  |  IK model
                // ---------+----------
                //   -PI/2  |   PI/2
                //     0    |    0
                //    PI/2  |  -PI/2
                //
                // ==> negative in MoveIt compared to what IK model is using
                //**********************
                aSolution.at( 2 ) = -jointAnglesVec.at( brIndex ).th4;

                //**********************
                // wrist joint
                //
                //  MoveIt  |  IK model
                // ---------+----------
                //   -PI/2  |   PI/2
                //     0    |    0
                //    PI/2  |  -PI/2
                //
                // ==> negative in MoveIt compared to what IK model is using
                //**********************
                aSolution.at( 3 ) = -jointAnglesVec.at( brIndex ).th5;

                RCLCPP_INFO( LOGGER, " => Returned solution: th2=%lf, th3=%lf, th4=%lf, th5=%lf \n",
                             convertRadToDeg( aSolution.at( 0 ) ),
                             convertRadToDeg( aSolution.at( 1 ) ),
                             convertRadToDeg( aSolution.at( 2 ) ),
                             convertRadToDeg( aSolution.at( 3 ) ) );
            }
        }
    }

    aErrorCode.val = status ? aErrorCode.SUCCESS : aErrorCode.NO_IK_SOLUTION;
    return status;
}


//!************************************************************************
//! Store joint angles so that next calculated ranks could be referenced
//! See computeIkm() and computeJointAnglesRank().
//!
//! @returns nothing
//!************************************************************************
void ADI_4DOF_KinematicsPlugin::updateJointAngles
    (
    const JointAngles aJointAngles  //!< joint angles
    ) const
{
    gJointAngles.th2 = aJointAngles.th2;
    gJointAngles.th3 = aJointAngles.th3;
    gJointAngles.th4 = aJointAngles.th4;
    gJointAngles.th5 = aJointAngles.th5;
}


//!************************************************************************
//! Validate a set of joint angles for a target end-effector position
//!
//! @returns true if the set of joint angles is valid
//!************************************************************************
bool ADI_4DOF_KinematicsPlugin::validateIkmSet
    (
    const double aTh2,  //!< angle of base joint [rad]
    const double aTh3,  //!< angle of shoulder joint [rad]
    const double aTh4,  //!< angle of elbow joint [rad]
    const double aTh5,  //!< angle of wrist joint [rad]
    const double aX,    //!< target end-effector x coordinate
    const double aY,    //!< target end-effector y coordinate
    const double aZ     //!< target end-effector z coordinate
    ) const
{
    bool status = false;

    double l1 = mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_LINK ).link.geometry.cylinder.length;
    double l2 = mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_COLUMN_LINK ).link.geometry.cylinder.length;
    double l3 = mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_UPPER_ARM_LINK ).link.geometry.cylinder.length;
    double l4 = mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_LOWER_ARM_LINK ).link.geometry.cylinder.length;
    double l5 = mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_END_EFFECTOR_LINK ).link.geometry.cylinder.length;

    double m = l3 * cos( aTh3 ) + l4 * cos( aTh3 + aTh4 ) + l5 * cos( aTh3 + aTh4 + aTh5 );
    double n = l3 * sin( aTh3 ) + l4 * sin( aTh3 + aTh4 ) + l5 * sin( aTh3 + aTh4 + aTh5 );

    double calcX = cos( aTh2 ) * m;
    double calcY = sin( aTh2 ) * m;
    double calcZ = l1 + l2 + n;

    status = ( fabs( aX - calcX ) < ZERO_L_THD )
          && ( fabs( aY - calcY ) < ZERO_L_THD )
          && ( fabs( aZ - calcZ ) < ZERO_L_THD );

    return status;
}

} // namespace

//!************************************************************************
//! Plugin class registration
//!************************************************************************
#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS( adi_4dof_ik_kinematics_plugin::ADI_4DOF_KinematicsPlugin, kinematics::KinematicsBase );

//#include <pluginlib/class_list_macros.hpp>
//PLUGINLIB_EXPORT_CLASS( adi_4dof_ik_kinematics_plugin::ADI_4DOF_KinematicsPlugin, kinematics::KinematicsBase );
