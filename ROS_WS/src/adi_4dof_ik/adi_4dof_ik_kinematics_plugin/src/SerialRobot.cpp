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
SerialRobot.cpp

This file contains the sources for the serial robot.
*/

#include <adi_4dof_ik/SerialRobot.hpp>

#include <cstring>


SerialRobot* SerialRobot::sInstance = nullptr;

const std::map<SerialRobot::PartsChain, std::string> SerialRobot::PARTS_CHAIN_NAMES =
{
    { SerialRobot::PARTS_CHAIN_BASE_LINK,                 "base_link" },
    { SerialRobot::PARTS_CHAIN_BASE_JOINT,                "base_joint" },
    { SerialRobot::PARTS_CHAIN_COLUMN_LINK,               "column_link" },
    { SerialRobot::PARTS_CHAIN_SHOULDER_JOINT,            "shoulder_joint" },
    { SerialRobot::PARTS_CHAIN_UPPER_ARM_LINK,            "upper_arm_link" },
    { SerialRobot::PARTS_CHAIN_ELBOW_JOINT,               "elbow_joint" },
    { SerialRobot::PARTS_CHAIN_LOWER_ARM_LINK,            "lower_arm_link" },
    { SerialRobot::PARTS_CHAIN_WRIST_JOINT,               "wrist_joint" },
    { SerialRobot::PARTS_CHAIN_END_EFFECTOR_LINK,         "end_effector_link" }
};

const SerialRobot::ColorRgba SerialRobot::COLOR_BLUE = { 0, 0, 0.8, 1 };
const SerialRobot::ColorRgba SerialRobot::COLOR_BLACK = { 0, 0, 0, 1 };


//!************************************************************************
//! Constructor
//!************************************************************************
SerialRobot::SerialRobot()
    : mTotalLength( 0 )
{
    memset( &mEndEffector, 0, sizeof( mEndEffector ) );
    initializeRobotParts();
}


//!************************************************************************
//! Singleton
//!
//! @returns the instance of the object
//!************************************************************************
SerialRobot* SerialRobot::getInstance()
{
    if( !sInstance )
    {
        sInstance = new SerialRobot;
    }

    return sInstance;
}


//!************************************************************************
//! Calculate the 3D coordinates of the end-effector
//!
//! @returns the end-effector coordinates
//!************************************************************************
SerialRobot::TriaxialPoint SerialRobot::calculateEndEffectorCoordinates()
{
    memset( &mEndEffector, 0, sizeof( mEndEffector ) );

    double th2 = mRobotPartsVec.at( PARTS_CHAIN_BASE_JOINT ).joint.thetaAngleRad;
    double th3 = mRobotPartsVec.at( PARTS_CHAIN_SHOULDER_JOINT ).joint.thetaAngleRad;
    double th4 = mRobotPartsVec.at( PARTS_CHAIN_ELBOW_JOINT ).joint.thetaAngleRad;
    double th5 = mRobotPartsVec.at( PARTS_CHAIN_WRIST_JOINT ).joint.thetaAngleRad;

    double l1 = mRobotPartsVec.at( PARTS_CHAIN_BASE_LINK ).link.geometry.cylinder.length;
    double l2 = mRobotPartsVec.at( PARTS_CHAIN_COLUMN_LINK ).link.geometry.cylinder.length;
    double l3 = mRobotPartsVec.at( PARTS_CHAIN_UPPER_ARM_LINK ).link.geometry.cylinder.length;
    double l4 = mRobotPartsVec.at( PARTS_CHAIN_LOWER_ARM_LINK ).link.geometry.cylinder.length;
    double l5 = mRobotPartsVec.at( PARTS_CHAIN_END_EFFECTOR_LINK ).link.geometry.cylinder.length;

    double m = l3 * cos( th3 ) + l4 * cos( th3 + th4 ) + l5 * cos( th3 + th4 + th5 );
    double n = l3 * sin( th3 ) + l4 * sin( th3 + th4 ) + l5 * sin( th3 + th4 + th5 );

    mEndEffector.x = cos( th2 ) * m;
    mEndEffector.y = sin( th2 ) * m;
    mEndEffector.z = l1 + l2 + n;

    return mEndEffector;
}


//!************************************************************************
//! Compute the mass of a link element, using an average density value
//!
//! @returns the link mass [kg]
//!************************************************************************
double SerialRobot::calculateLinkMass
    (
    const Link      aLink,      //!< serial robot link
    const double    aAvgDensity //!< average density [kg/m3]
    )
{
    double mass = 0;

    if( aAvgDensity > 0 )
    {
        mass = aAvgDensity;

        switch( aLink.geometricShape )
        {
            case GEOMETRIC_SHAPE_CYLINDER:
                mass *= PI * aLink.geometry.cylinder.length * pow( aLink.geometry.cylinder.radius, 2.0 );
                break;

            case GEOMETRIC_SHAPE_PARALLELEPIPED:
                mass *= aLink.geometry.box.xSize * aLink.geometry.box.ySize * aLink.geometry.box.zSize;
                break;

            default:
                break;
        }
    }

    return mass;
}


//!************************************************************************
//! Calculate the inertia tensor for a link
//!
//! @returns nothing
//!************************************************************************
void SerialRobot::calculateInertiaTensor
    (
    Link& aLink             //!< link
    )
{
    InertiaTensor inertiaTensor;
    memset( &inertiaTensor, 0, sizeof( inertiaTensor ) );

    switch( aLink.geometricShape )
    {
        case GEOMETRIC_SHAPE_CYLINDER:
            {
                double t1 = 0.5 * aLink.mass * pow( aLink.geometry.cylinder.radius, 2.0 );
                double t2 = 0.5 * t1 + aLink.mass * pow( aLink.geometry.cylinder.length, 2.0 ) / 12.0;
                inertiaTensor[0][0] = t2;
                inertiaTensor[1][1] = t2;
                inertiaTensor[2][2] = t1;
            }
            break;

        case GEOMETRIC_SHAPE_PARALLELEPIPED:
            {
                double t = aLink.mass / 12.0;
                double ixx = pow( aLink.geometry.box.ySize, 2.0 ) + pow( aLink.geometry.box.zSize, 2.0 );
                double iyy = pow( aLink.geometry.box.xSize, 2.0 ) + pow( aLink.geometry.box.zSize, 2.0 );
                double izz = pow( aLink.geometry.box.xSize, 2.0 ) + pow( aLink.geometry.box.ySize, 2.0 );
                inertiaTensor[0][0] = ixx * t;
                inertiaTensor[1][1] = iyy * t;
                inertiaTensor[2][2] = izz * t;
            }
            break;

        default:
            break;
    }

    memcpy( &aLink.inertiaTensor, &inertiaTensor, sizeof( inertiaTensor ) );
}


//!************************************************************************
//! Initialize the robot parts
//!
//! @returns nothing
//!************************************************************************
void SerialRobot::clearRobotPart
    (
    RobotPart&  aRobotPart  //!< robot part
    )
{
    aRobotPart.partType = PART_UNKNOWN;
    memset( &aRobotPart.link, 0, sizeof( aRobotPart.link ) );
    memset( &aRobotPart.joint, 0, sizeof( aRobotPart.joint ) );
}


//!************************************************************************
//! Perform a sanity check on the robot parts
//!
//! @returns true if sanity ckeck passes
//!************************************************************************
bool SerialRobot::doSanityCheckRobotParts()
{
    bool status = true;
    double diff = 0;
    const double EPS = 1.e-7;

    //************************
    // 0. LINK - base
    //************************
    diff = mRobotPartsVec.at( PARTS_CHAIN_BASE_LINK ).link.origin.z
           - 0.5 * mRobotPartsVec.at( PARTS_CHAIN_BASE_LINK ).link.geometry.cylinder.length;
    status = fabs( diff ) < EPS;

    //************************
    // 2. LINK - column
    //************************
    if( status )
    {
        diff = mRobotPartsVec.at( PARTS_CHAIN_COLUMN_LINK ).link.origin.z
               - mRobotPartsVec.at( PARTS_CHAIN_BASE_LINK ).link.geometry.cylinder.length;
        status = fabs( diff ) < EPS;
    }

    //************************
    // 1. JOINT - base
    //************************
    if( status )
    {
        diff = mRobotPartsVec.at( PARTS_CHAIN_BASE_JOINT ).joint.origin.z
               - 0.5 * mRobotPartsVec.at( PARTS_CHAIN_COLUMN_LINK ).link.geometry.cylinder.length;
        status = fabs( diff ) < EPS;
    }

    //************************
    // 4. LINK - upper arm
    //************************
    if( status )
    {
        diff = mRobotPartsVec.at( PARTS_CHAIN_UPPER_ARM_LINK ).link.origin.z
               - 0.5 *  mRobotPartsVec.at( PARTS_CHAIN_UPPER_ARM_LINK ).link.geometry.cylinder.length;
        status = fabs( diff ) < EPS;
    }

    //************************
    // 3. JOINT - shoulder
    //************************
    if( status )
    {
        diff = mRobotPartsVec.at( PARTS_CHAIN_SHOULDER_JOINT ).joint.origin.z
               - ( mRobotPartsVec.at( PARTS_CHAIN_BASE_JOINT ).joint.origin.z + mRobotPartsVec.at( PARTS_CHAIN_COLUMN_LINK ).link.origin.z );
        status = fabs( diff ) < EPS;
    }

    //************************
    // 6. LINK - lower arm
    //************************
    if( status )
    {
        diff = mRobotPartsVec.at( PARTS_CHAIN_LOWER_ARM_LINK ).link.origin.z
               - 0.5 *  mRobotPartsVec.at( PARTS_CHAIN_LOWER_ARM_LINK ).link.geometry.cylinder.length;
        status = fabs( diff ) < EPS;
    }

    //************************
    // 5. JOINT - elbow
    //************************
    if( status )
    {
        diff = mRobotPartsVec.at( PARTS_CHAIN_ELBOW_JOINT ).joint.origin.z
               - mRobotPartsVec.at( PARTS_CHAIN_UPPER_ARM_LINK ).link.geometry.cylinder.length;
        status = fabs( diff ) < EPS;
    }

    //************************
    // 8. LINK - end effector
    //************************
    if( status )
    {
        diff = mRobotPartsVec.at( PARTS_CHAIN_END_EFFECTOR_LINK ).link.origin.z
               - 0.5 *  mRobotPartsVec.at( PARTS_CHAIN_END_EFFECTOR_LINK ).link.geometry.cylinder.length;
        status = fabs( diff ) < EPS;
    }

    //************************
    // 7. JOINT - wrist
    //************************
    if( status )
    {
        diff = mRobotPartsVec.at( PARTS_CHAIN_WRIST_JOINT ).joint.origin.z
               - mRobotPartsVec.at( PARTS_CHAIN_LOWER_ARM_LINK ).link.geometry.cylinder.length;
        status = fabs( diff ) < EPS;
    }

    return status;
}


//!************************************************************************
//! Get the 3D coordinates of the end-effector
//!
//! @returns the end-effector coordinates
//!************************************************************************
SerialRobot::TriaxialPoint& SerialRobot::getEndEffectorCoordinates()
{
    return mEndEffector;
}


//!************************************************************************
//! Get the robot parts vector
//!
//! @returns The vector with robot parts
//!************************************************************************
std::vector<SerialRobot::RobotPart>& SerialRobot::getRobotPartsVec()
{
    return mRobotPartsVec;
}


//!************************************************************************
//! Get the total length of the robot, considering all links are colinear
//! (in extension)
//!
//! @returns The sum of all links' lengths
//!************************************************************************
double SerialRobot::getTotalLength()
{
    mTotalLength = 0;

    for( size_t i = 0; i < mRobotPartsVec.size(); i++ )
    {
        if( PART_LINK == mRobotPartsVec.at( i ).partType )
        {
            switch( mRobotPartsVec.at( i ).link.geometricShape )
            {
                case GEOMETRIC_SHAPE_CYLINDER:
                    mTotalLength += mRobotPartsVec.at( i ).link.geometry.cylinder.length;
                    break;

                case GEOMETRIC_SHAPE_PARALLELEPIPED:
                    mTotalLength += mRobotPartsVec.at( i ).link.geometry.box.zSize;
                    break;

                default:
                    break;
            }
        }
    }

    return mTotalLength;
}


//!************************************************************************
//! Initialize the robot parts
//!
//! @returns nothing
//!************************************************************************
void SerialRobot::initializeRobotParts()
{
    mRobotPartsVec.clear();
    mRobotPartsVec.resize( PARTS_CHAIN_COUNT );

    RobotPart robotPart;

    //************************
    // 0. LINK - base
    //************************
    clearRobotPart( robotPart );

    robotPart.partType = PART_LINK;
    robotPart.link.name = PARTS_CHAIN_NAMES.at( PARTS_CHAIN_BASE_LINK );

    robotPart.link.geometricShape = GEOMETRIC_SHAPE_CYLINDER;
    robotPart.link.geometry.cylinder.length = 0.05;
    robotPart.link.geometry.cylinder.radius = 0.05;

    robotPart.link.origin.z = 0.5 * robotPart.link.geometry.cylinder.length;

    robotPart.link.material = MATERIAL_BLUE;

    if( USE_FIXED_BASE_MASS )
    {
        robotPart.link.mass = 10.0;
    }
    else
    {
        robotPart.link.mass = calculateLinkMass( robotPart.link, AVG_DENSITY );
    }

    calculateInertiaTensor( robotPart.link );

    mRobotPartsVec.at( PARTS_CHAIN_BASE_LINK ) = robotPart;

    //************************
    // 2. LINK - column
    //************************
    clearRobotPart( robotPart );

    robotPart.partType = PART_LINK;
    robotPart.link.name = PARTS_CHAIN_NAMES.at( PARTS_CHAIN_COLUMN_LINK );

    robotPart.link.geometricShape = GEOMETRIC_SHAPE_CYLINDER;
    robotPart.link.geometry.cylinder.length = 0.16;
    robotPart.link.geometry.cylinder.radius = 0.025;

    robotPart.link.origin.z = mRobotPartsVec.at( PARTS_CHAIN_BASE_LINK ).link.geometry.cylinder.length;

    robotPart.link.material = MATERIAL_BLUE;

    robotPart.link.mass = calculateLinkMass( robotPart.link, AVG_DENSITY );
    calculateInertiaTensor( robotPart.link );

    mRobotPartsVec.at( PARTS_CHAIN_COLUMN_LINK ) = robotPart;

    //************************
    // 1. JOINT - base
    //************************
    clearRobotPart( robotPart );

    robotPart.partType = PART_JOINT;
    robotPart.joint.name = PARTS_CHAIN_NAMES.at( PARTS_CHAIN_BASE_JOINT );
    robotPart.joint.type = JOINT_TYPE_REVOLUTE;

    robotPart.joint.parentLink = mRobotPartsVec.at( PARTS_CHAIN_BASE_LINK ).link;
    robotPart.joint.childLink = mRobotPartsVec.at( PARTS_CHAIN_COLUMN_LINK ).link;

    robotPart.joint.orientation.zAxis = 1;
    robotPart.joint.origin.z = 0.5 * mRobotPartsVec.at( PARTS_CHAIN_COLUMN_LINK ).link.geometry.cylinder.length;

    robotPart.joint.limit.effort = 100.0;   // %
    robotPart.joint.limit.velocity = 0.5;   // rad/s

    robotPart.joint.limit.lowerAngleRad = MIN_BASE_JOINT_ANGLE_RAD;
    robotPart.joint.limit.upperAngleRad = MAX_BASE_JOINT_ANGLE_RAD;

    robotPart.joint.thetaAngleRad = 0;

    // XRDF-only
    robotPart.joint.accelerationLimit = 12;
    robotPart.joint.jerkLimit = 500;

    mRobotPartsVec.at( PARTS_CHAIN_BASE_JOINT ) = robotPart;

    //************************
    // 4. LINK - upper arm
    //************************
    clearRobotPart( robotPart );

    robotPart.partType = PART_LINK;
    robotPart.link.name = PARTS_CHAIN_NAMES.at( PARTS_CHAIN_UPPER_ARM_LINK );

    robotPart.link.geometricShape = GEOMETRIC_SHAPE_CYLINDER;
    robotPart.link.geometry.cylinder.length = 0.55;
    robotPart.link.geometry.cylinder.radius = 0.02;

    robotPart.link.origin.z = 0.5 * robotPart.link.geometry.cylinder.length;

    robotPart.link.material = MATERIAL_BLUE;

    robotPart.link.mass = calculateLinkMass( robotPart.link, AVG_DENSITY );
    calculateInertiaTensor( robotPart.link );

    mRobotPartsVec.at( PARTS_CHAIN_UPPER_ARM_LINK ) = robotPart;

    //************************
    // 3. JOINT - shoulder
    //************************
    clearRobotPart( robotPart );

    robotPart.partType = PART_JOINT;
    robotPart.joint.name = PARTS_CHAIN_NAMES.at( PARTS_CHAIN_SHOULDER_JOINT );

    robotPart.joint.type = JOINT_TYPE_REVOLUTE;
    robotPart.joint.parentLink = mRobotPartsVec.at( PARTS_CHAIN_COLUMN_LINK ).link;
    robotPart.joint.childLink = mRobotPartsVec.at( PARTS_CHAIN_UPPER_ARM_LINK ).link;

    robotPart.joint.orientation.yAxis = 1;
    robotPart.joint.origin.z = mRobotPartsVec.at( PARTS_CHAIN_BASE_JOINT ).joint.origin.z
                             + mRobotPartsVec.at( PARTS_CHAIN_COLUMN_LINK ).link.origin.z;

    robotPart.joint.limit.effort = 81.0;    // %
    robotPart.joint.limit.velocity = 0.5;   // rad/s

    robotPart.joint.limit.lowerAngleRad = MIN_SHOULDER_JOINT_ANGLE_RAD;
    robotPart.joint.limit.upperAngleRad = MAX_SHOULDER_JOINT_ANGLE_RAD;

    robotPart.joint.thetaAngleRad = 0.5 * PI;

    // XRDF-only
    robotPart.joint.accelerationLimit = 12;
    robotPart.joint.jerkLimit = 500;

    mRobotPartsVec.at( PARTS_CHAIN_SHOULDER_JOINT ) = robotPart;

    //************************
    // 6. LINK - lower arm
    //************************
    clearRobotPart( robotPart );

    robotPart.partType = PART_LINK;
    robotPart.link.name = PARTS_CHAIN_NAMES.at( PARTS_CHAIN_LOWER_ARM_LINK );

    robotPart.link.geometricShape = GEOMETRIC_SHAPE_CYLINDER;
    robotPart.link.geometry.cylinder.length = 0.55;
    robotPart.link.geometry.cylinder.radius = 0.015;

    robotPart.link.origin.z = 0.5 * robotPart.link.geometry.cylinder.length;

    robotPart.link.material = MATERIAL_BLUE;

    robotPart.link.mass = calculateLinkMass( robotPart.link, AVG_DENSITY );
    calculateInertiaTensor( robotPart.link );

    mRobotPartsVec.at( PARTS_CHAIN_LOWER_ARM_LINK ) = robotPart;

    //************************
    // 5. JOINT - elbow
    //************************
    clearRobotPart( robotPart );

    robotPart.partType = PART_JOINT;
    robotPart.joint.name = PARTS_CHAIN_NAMES.at( PARTS_CHAIN_ELBOW_JOINT );

    robotPart.joint.type = JOINT_TYPE_REVOLUTE;
    robotPart.joint.parentLink = mRobotPartsVec.at( PARTS_CHAIN_UPPER_ARM_LINK ).link;
    robotPart.joint.childLink = mRobotPartsVec.at( PARTS_CHAIN_LOWER_ARM_LINK ).link;

    robotPart.joint.orientation.yAxis = 1;
    robotPart.joint.origin.z = mRobotPartsVec.at( PARTS_CHAIN_UPPER_ARM_LINK ).link.geometry.cylinder.length;

    robotPart.joint.limit.effort = 81.0;    // %
    robotPart.joint.limit.velocity = 0.5;   // rad/s

    robotPart.joint.limit.lowerAngleRad = MIN_ELBOW_JOINT_ANGLE_RAD;
    robotPart.joint.limit.upperAngleRad = MAX_ELBOW_JOINT_ANGLE_RAD;

    robotPart.joint.thetaAngleRad = 0;

    // XRDF-only
    robotPart.joint.accelerationLimit = 12;
    robotPart.joint.jerkLimit = 500;

    mRobotPartsVec.at( PARTS_CHAIN_ELBOW_JOINT ) = robotPart;

    //************************
    // 8. LINK - end effector
    //************************
    clearRobotPart( robotPart );

    robotPart.partType = PART_LINK;
    robotPart.link.name = PARTS_CHAIN_NAMES.at( PARTS_CHAIN_END_EFFECTOR_LINK );

    robotPart.link.geometricShape = GEOMETRIC_SHAPE_CYLINDER;
    robotPart.link.geometry.cylinder.length = 0.09;
    robotPart.link.geometry.cylinder.radius = 0.01;

    robotPart.link.origin.z = 0.5 * robotPart.link.geometry.cylinder.length;

    robotPart.link.material = MATERIAL_BLUE;

    robotPart.link.mass = calculateLinkMass( robotPart.link, AVG_DENSITY );
    calculateInertiaTensor( robotPart.link );

    mRobotPartsVec.at( PARTS_CHAIN_END_EFFECTOR_LINK ) = robotPart;

    //************************
    // 7. JOINT - wrist
    //************************
    clearRobotPart( robotPart );

    robotPart.partType = PART_JOINT;
    robotPart.joint.name = PARTS_CHAIN_NAMES.at( PARTS_CHAIN_WRIST_JOINT );

    robotPart.joint.type = JOINT_TYPE_REVOLUTE;
    robotPart.joint.parentLink = mRobotPartsVec.at( PARTS_CHAIN_LOWER_ARM_LINK ).link;
    robotPart.joint.childLink = mRobotPartsVec.at( PARTS_CHAIN_END_EFFECTOR_LINK ).link;

    robotPart.joint.orientation.yAxis = 1;
    robotPart.joint.origin.z = mRobotPartsVec.at( PARTS_CHAIN_LOWER_ARM_LINK ).link.geometry.cylinder.length;

    robotPart.joint.limit.effort = 81.0;    // %
    robotPart.joint.limit.velocity = 0.5;   // rad/s

    robotPart.joint.limit.lowerAngleRad = MIN_WRIST_JOINT_ANGLE_RAD;
    robotPart.joint.limit.upperAngleRad = MAX_WRIST_JOINT_ANGLE_RAD;

    robotPart.joint.thetaAngleRad = 0;

    // XRDF-only
    robotPart.joint.accelerationLimit = 12;
    robotPart.joint.jerkLimit = 500;

    mRobotPartsVec.at( PARTS_CHAIN_WRIST_JOINT ) = robotPart;


    //************************
    // End effector coordinates
    //************************
    calculateEndEffectorCoordinates();
}
