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
XrdfHandler.cpp

This file contains the sources for the Nvidia XRDF handler.
*/

#include "XrdfHandler.h"

#include <QFileDialog>
#include <QMessageBox>

#include <cmath>


//!************************************************************************
//! Constructor
//!************************************************************************
XrdfHandler::XrdfHandler()
{
}


//!************************************************************************
//! Customize the string containing a coordinate value.
//! It avoids returning numbers without decimal part, even if zero.
//!
//! @returns QString representing the value
//!************************************************************************
QString XrdfHandler::customizeCoordinateStr
    (
    const double aCoord     //!< coordinate
    ) const
{
    QString retStr;
    const double EPS = 1.e-5;

    if( fabs( aCoord ) < EPS )
    {
        retStr = "0.0";
    }
    else if( fabs( aCoord ) == fabs( static_cast<int>( aCoord ) ) )
    {
        retStr = QString::number( aCoord ) + ".0";
    }
    else
    {
        retStr = QString::number( aCoord );
    }

    return retStr;
}


//!************************************************************************
//! Generate a XRDF file
//! Entire content is based on YAML 1.2.
//!
//! @returns nothing
//!************************************************************************
void XrdfHandler::generateFile()
{
    const QString DEFAULT_FILENAME = "adi_robotarm.xrdf";
    QString selectedFilter;

    QString fileName = QFileDialog::getSaveFileName( this,
                                                     "Save XRDF",
                                                     DEFAULT_FILENAME,
                                                     "XRDF files (*.xrdf)",
                                                     &selectedFilter,
                                                     QFileDialog::DontUseNativeDialog
                                                    );
    if( !fileName.isEmpty() )
    {
        std::string outputFilename = fileName.toStdString();
        mOutputFile.open( outputFilename );

        if( mOutputFile.is_open() )
        {            
            writeFormatVersion();
            writeModifiers();
            writeDefaultJointPositions();
            writeCspace();
            writeToolFrames();
            writeCollision();
            writeSelfCollision();
            writeGeometry();

            mOutputFile.close();
        }
        else if( fileName.size() )
        {
            QString msg = "Could not open file \"" + fileName +"\".";
            QMessageBox msgBox;
            msgBox.setText( msg );
            msgBox.exec();
        }
    }
}


//!************************************************************************
//! Get the base frame
//!
//! @returns A string with the base frame
//!************************************************************************
std::string XrdfHandler::getBaseFrame() const
{
    std::string baseFrameString;

    if( SerialRobot::PARTS_CHAIN_COUNT == mRobotPartsVec.size() )
    {
        baseFrameString = mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_LINK ).link.name;
    }

    return baseFrameString;
}


//!************************************************************************
//! Get the geometry name
//!
//! @returns A string with the geometry name
//!************************************************************************
std::string XrdfHandler::getGeometryName() const
{
    const std::string ROBOT_NAME = "adi_robotarm";
    const std::string GEOMETRY_NAME_SUFFIX = "_collision_spheres";
    std::string geometryName = ROBOT_NAME + GEOMETRY_NAME_SUFFIX;

    return geometryName;
}


//!************************************************************************
//! Set the indent level by inserting spaces
//!
//! @returns The indent string
//!************************************************************************
std::string XrdfHandler::indent
    (
    const uint8_t aLevel    //!< level of indent
    ) const
{
    std::string indentString;

    for( uint8_t i = 0; i < aLevel; i++ )
    {
        indentString += "  ";
    }

    return indentString;
}


//!************************************************************************
//! Set the vector with robot parts
//!
//! @returns nothing
//!************************************************************************
void XrdfHandler::setRobotPartsVec
    (
    const std::vector<SerialRobot::RobotPart>   aRobotPartsVec      //!< robot parts vector
    )
{
    mRobotPartsVec = aRobotPartsVec;
}


//!************************************************************************
//! Write the collision in file
//! It referrs to collisions between the robot and environment.
//! https://nvidia-isaac-ros.github.io/concepts/manipulation/xrdf.html#collision-geometry-optional
//!
//! @returns nothing
//!************************************************************************
void XrdfHandler::writeCollision()
{
    std::string contentString = "collision:\n";

    contentString += indent( 1 );
    contentString += "geometry: ";
    contentString += "\"";
    contentString += getGeometryName();
    contentString += "\"\n";

    contentString += indent( 1 );
    contentString += "buffer_distance:\n";

    for( size_t i = 0; i < mRobotPartsVec.size(); i++ )
    {
        // base link is fixed with respect to the environment, so no collision is possible
        if( SerialRobot::PARTS_CHAIN_BASE_LINK == i )
        {
            continue;
        }

        if( SerialRobot::PART_LINK == mRobotPartsVec.at( i ).partType  )
        {
            contentString += indent( 2 );
            contentString += mRobotPartsVec.at( i ).link.name;
            contentString += ": ";
            contentString += QString::number( SerialRobot::ANTI_COLLISION_DIST, 'f', 3 ).toStdString();
            contentString += "\n";
        }
    }

    contentString += "\n";
    mOutputFile << contentString;
}


//!************************************************************************
//! Write the cspace in file
//! https://nvidia-isaac-ros.github.io/concepts/manipulation/xrdf.html#c-space-definition-required
//!
//! @returns nothing
//!************************************************************************
void XrdfHandler::writeCspace()
{
    std::string contentString = "cspace:\n";

    contentString += indent( 1 );
    contentString += "joint_names:\n";

    for( size_t i = 0; i < mRobotPartsVec.size(); i++ )
    {
        if( SerialRobot::PART_JOINT == mRobotPartsVec.at( i ).partType  )
        {
            if( SerialRobot::JOINT_TYPE_FIXED != mRobotPartsVec.at( i ).joint.type )
            {
                contentString += indent( 2 );
                contentString += "- \"";
                contentString += mRobotPartsVec.at( i ).joint.name;
                contentString += "\"\n";
            }
        }
    }

    contentString += indent( 1 );
    contentString += "acceleration_limits: [";
    bool firstItem = true;

    for( size_t i = 0; i < mRobotPartsVec.size(); i++ )
    {
        if( SerialRobot::PART_JOINT == mRobotPartsVec.at( i ).partType  )
        {
            if( SerialRobot::JOINT_TYPE_FIXED != mRobotPartsVec.at( i ).joint.type )
            {
                contentString += firstItem ? "" : ", ";
                contentString += QString::number( mRobotPartsVec.at( i ).joint.accelerationLimit, 'f', 1 ).toStdString();
                firstItem = false;
            }
        }
    }

    contentString += "]\n";

    contentString += indent( 1 );
    contentString += "jerk_limits: [";
    firstItem = true;

    for( size_t i = 0; i < mRobotPartsVec.size(); i++ )
    {
        if( SerialRobot::PART_JOINT == mRobotPartsVec.at( i ).partType  )
        {
            if( SerialRobot::JOINT_TYPE_FIXED != mRobotPartsVec.at( i ).joint.type )
            {
                contentString += firstItem ? "" : ", ";
                contentString += QString::number( mRobotPartsVec.at( i ).joint.jerkLimit, 'f', 1 ).toStdString();
                firstItem = false;
            }
        }
    }

    contentString += "]\n";

    contentString += "\n";
    mOutputFile << contentString;
}


//!************************************************************************
//! Write the default joint positions in file
//! https://nvidia-isaac-ros.github.io/concepts/manipulation/xrdf.html#default-joint-positions-optional
//!
//! @returns nothing
//!************************************************************************
void XrdfHandler::writeDefaultJointPositions()
{
    std::string contentString = "default_joint_positions:\n";

    for( size_t i = 0; i < mRobotPartsVec.size(); i++ )
    {
        if( SerialRobot::PART_JOINT == mRobotPartsVec.at( i ).partType  )
        {
            if( SerialRobot::JOINT_TYPE_FIXED != mRobotPartsVec.at( i ).joint.type )
            {
                contentString += indent( 1 );
                contentString += mRobotPartsVec.at( i ).joint.name;
                contentString += ": ";
                contentString += QString::number( mRobotPartsVec.at( i ).joint.defaultPosition, 'f', 1 ).toStdString();
                contentString += "\n";
            }
        }
    }

    contentString += "\n";
    mOutputFile << contentString;
}


//!************************************************************************
//! Write the format version in file
//! https://nvidia-isaac-ros.github.io/concepts/manipulation/xrdf.html#format-version-required
//!
//! @returns nothing
//!************************************************************************
void XrdfHandler::writeFormatVersion()
{
    QString contentString = "format: xrdf\n";
    contentString += "format_version: 1.0\n";

    contentString += "\n";
    mOutputFile << contentString.toStdString();
}


//!************************************************************************
//! Write the geometry in file
//! https://nvidia-isaac-ros.github.io/concepts/manipulation/xrdf.html#geometry-groups-optional
//!
//! @returns nothing
//!************************************************************************
void XrdfHandler::writeGeometry()
{
    std::string contentString = "geometry:\n";

    contentString += indent( 1 );
    contentString += getGeometryName();
    contentString += ":\n";

    contentString += indent( 2 );
    contentString += "spheres:\n";

    for( size_t i = 0; i < mRobotPartsVec.size(); i++ )
    {
        if( SerialRobot::PART_LINK == mRobotPartsVec.at( i ).partType  )
        {
            contentString += indent( 3 );
            contentString += mRobotPartsVec.at( i ).link.name;
            contentString += ":\n";

            double posMin = 0;
            double posMax = 0;
            double posStep = 0;

            switch( mRobotPartsVec.at( i ).link.geometricShape )
            {
                case SerialRobot::GEOMETRIC_SHAPE_CYLINDER:
                    posMin = mRobotPartsVec.at( i ).link.origin.z - 0.4 * mRobotPartsVec.at( i ).link.geometry.cylinder.length;
                    posMax = mRobotPartsVec.at( i ).link.origin.z + 0.4 * mRobotPartsVec.at( i ).link.geometry.cylinder.length;
                    posStep = mRobotPartsVec.at( i ).link.geometry.cylinder.radius;
                    break;

                case SerialRobot::GEOMETRIC_SHAPE_PARALLELEPIPED:
                    posMin = mRobotPartsVec.at( i ).link.origin.z - mRobotPartsVec.at( i ).link.geometry.box.zSize;
                    posMax = mRobotPartsVec.at( i ).link.origin.z + mRobotPartsVec.at( i ).link.geometry.box.zSize;
                    posStep = 0.5 * std::max( mRobotPartsVec.at( i ).link.geometry.box.xSize,
                                              mRobotPartsVec.at( i ).link.geometry.box.ySize );
                    break;

                default:
                    break;
            }

            ssize_t sphereCount = ( posMax - posMin ) / posStep;

            if( sphereCount < 1 )
            {
                sphereCount = 1;
            }

            for( size_t j = 0; j < sphereCount; j++ )
            {
                // sphere center
                contentString += indent( 4 );
                contentString += "- center: [";
                contentString += customizeCoordinateStr( mRobotPartsVec.at( i ).link.origin.x ).toStdString();
                contentString += ", ";
                contentString += customizeCoordinateStr( mRobotPartsVec.at( i ).link.origin.y ).toStdString();
                contentString += ", ";
                contentString += customizeCoordinateStr( posMin + j * posStep ).toStdString();
                contentString += "]\n";

                // sphere radius
                contentString += indent( 4 );
                contentString += "  radius: ";
                double sphereRadius = 0;

                switch( mRobotPartsVec.at( i ).link.geometricShape )
                {
                    case SerialRobot::GEOMETRIC_SHAPE_CYLINDER:
                        sphereRadius = mRobotPartsVec.at( i ).link.geometry.cylinder.radius;
                        break;

                    case SerialRobot::GEOMETRIC_SHAPE_PARALLELEPIPED:
                        sphereRadius = std::max( mRobotPartsVec.at( i ).link.geometry.box.xSize,
                                                 mRobotPartsVec.at( i ).link.geometry.box.ySize );
                        break;

                    default:
                        break;
                }

                contentString += QString::number( sphereRadius ).toStdString();
                contentString += "\n";
            }
        }
    }

    contentString += "\n";
    mOutputFile << contentString;
}


//!************************************************************************
//! Write the URDF modifiers in file
//! https://nvidia-isaac-ros.github.io/concepts/manipulation/xrdf.html#urdf-modifiers-optional
//!
//! @returns nothing
//!************************************************************************
void XrdfHandler::writeModifiers()
{
    std::string contentString = "modifiers:\n";

    contentString += indent( 1 );
    contentString += "- set_base_frame: ";
    contentString += "\"";
    contentString += getBaseFrame();
    contentString += "\"";
    contentString += "\n";

    contentString += "\n";
    mOutputFile << contentString;
}


//!************************************************************************
//! Write the self_collision in file
//! It referrs to collisions between mechanical components belonging to the
//! robot.
//! https://nvidia-isaac-ros.github.io/concepts/manipulation/xrdf.html#self-collision-geometry-optional
//!
//! @returns nothing
//!************************************************************************
void XrdfHandler::writeSelfCollision()
{
    std::string contentString = "self_collision:\n";

    contentString += indent( 1 );
    contentString += "geometry: ";
    contentString += "\"";
    contentString += getGeometryName();
    contentString += "\"\n";

    contentString += indent( 1 );
    contentString += "buffer_distance:\n";

    for( size_t i = 0; i < mRobotPartsVec.size(); i++ )
    {
        if( SerialRobot::PART_LINK == mRobotPartsVec.at( i ).partType  )
        {
            contentString += indent( 2 );
            contentString += mRobotPartsVec.at( i ).link.name;
            contentString += ": ";
            // use only half of the defined distance, since buffers applied to interacting links are additive
            contentString += QString::number( 0.5 * SerialRobot::ANTI_COLLISION_DIST, 'f', 3 ).toStdString();
            contentString += "\n";
        }
    }

    contentString += indent( 1 );
    contentString += "ignore:\n";

    // base and column links cannot collide since are always perpendicular
    contentString += indent( 2 );
    contentString += "\"";
    contentString += mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_LINK ).link.name;
    contentString += "\": [\"";
    contentString += mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_COLUMN_LINK ).link.name;
    contentString += "\"]\n";

    for( size_t i = 0; i < mRobotPartsVec.size(); i++ )
    {
        if( SerialRobot::PART_JOINT == mRobotPartsVec.at( i ).partType )
        {
            // links adjacent to a fixed joint cannot collide
            if( SerialRobot::JOINT_TYPE_FIXED == mRobotPartsVec.at( i ).joint.type )
            {
                contentString += indent( 2 );
                contentString += "\"";
                contentString += mRobotPartsVec.at( i ).joint.parentLink.name;
                contentString += "\": [\"";
                contentString += mRobotPartsVec.at( i ).joint.childLink.name;
                contentString += "\"]\n";
            }
        }
    }

    contentString += "\n";
    mOutputFile << contentString;
}


//!************************************************************************
//! Write the tool_frames in file
//! https://nvidia-isaac-ros.github.io/concepts/manipulation/xrdf.html#tool-frame-definition-optional
//!
//! @returns nothing
//!************************************************************************
void XrdfHandler::writeToolFrames()
{
    std::string contentString = "tool_frames: [\"";
    contentString += mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_END_EFFECTOR_LINK ).link.name;
    contentString += "\"]\n";

    contentString += "\n";
    mOutputFile << contentString;
}
