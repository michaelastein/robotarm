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
UrdfHandler.cpp

This file contains the sources for the URDF handler.
*/

#include "UrdfHandler.h"

#include <QByteArray>
#include <QDomDocument>
#include <QDomNode>
#include <QFile>
#include <QFileDialog>
#include <QMessageBox>
#include <QString>
#include <QTextStream>


const std::map<SerialRobot::Part, QString> UrdfHandler::PART_NAMES =
{
    { SerialRobot::PART_LINK,  "link" },
    { SerialRobot::PART_JOINT, "joint" }
};

const std::map<SerialRobot::JointType, QString> UrdfHandler::JOINT_TYPE_NAMES =
{
    { SerialRobot::JOINT_TYPE_CONTINUOUS, "continuous" },
    { SerialRobot::JOINT_TYPE_FIXED,      "fixed" },
    { SerialRobot::JOINT_TYPE_PRISMATIC,  "prismatic" },
    { SerialRobot::JOINT_TYPE_REVOLUTE,   "revolute" }
};

const std::map<SerialRobot::GeometricShape, QString> UrdfHandler::GEOMETRIC_SHAPE_NAMES =
{
    { SerialRobot::GEOMETRIC_SHAPE_CYLINDER,       "cylinder" },
    { SerialRobot::GEOMETRIC_SHAPE_PARALLELEPIPED, "box" }
};

const std::map<SerialRobot::Material, QString> UrdfHandler::MATERIAL_NAMES =
{
    { SerialRobot::MATERIAL_BLUE,  "blue" },
    { SerialRobot::MATERIAL_BLACK, "black" }
};

//!************************************************************************
//! Constructor
//!************************************************************************
UrdfHandler::UrdfHandler()
{
}


//!************************************************************************
//! Create a formatted URDF comment using a string
//!
//! @returns The formatted string
//!************************************************************************
QString UrdfHandler::createCommentString
    (
    const std::string aString   //!< comment string
    ) const
{
    std::string commentStr = "\n";

    for( uint8_t i = 0; i < COMMENT_LINE_LEN; i++ )
    {
        commentStr += "*";
    }

    commentStr += "\n ";
    commentStr += aString;
    commentStr += "\n";

    for( uint8_t i = 0; i < COMMENT_LINE_LEN; i++ )
    {
        commentStr += "*";
    }

    commentStr += "\n ";

    return QString::fromStdString( commentStr );
}


//!************************************************************************
//! Create a formatted URDF comment using a robot part
//!
//! @returns The formatted string
//!************************************************************************
QString UrdfHandler::createCommentString
    (
    const SerialRobot::RobotPart aRobotPart //!< robot part
    ) const
{
    std::string commentStr = "\n";

    for( uint8_t i = 0; i < COMMENT_LINE_LEN; i++ )
    {
        commentStr += "*";
    }

    commentStr += "\n ";

    switch( aRobotPart.partType )
    {
        case SerialRobot::PART_LINK:
            commentStr += "LINK - ";
            commentStr += aRobotPart.link.name;
            break;

        case SerialRobot::PART_JOINT:
            commentStr += "JOINT - ";
            commentStr += aRobotPart.joint.name;
            break;

        default:
            break;
    }

    commentStr += "\n";

    for( uint8_t i = 0; i < COMMENT_LINE_LEN; i++ )
    {
        commentStr += "*";
    }

    commentStr += "\n ";

    return QString::fromStdString( commentStr );
}


//!************************************************************************
//! Create a string with RGBA values of a color
//!
//! @returns The string with RGBA values
//!************************************************************************
QString UrdfHandler::createRgbaString
    (
    const SerialRobot::ColorRgba aColor //!< RGBA color
    ) const
{
    QString rgbaStr = QString::number( aColor.red ) + " " +
                      QString::number( aColor.green ) + " " +
                      QString::number( aColor.blue ) + " " +
                      QString::number( aColor.alpha );
    return rgbaStr;
}


//!************************************************************************
//! Generate an URDF file
//!
//! @returns nothing
//!************************************************************************
void UrdfHandler::generateFile()
{
    const QString DEFAULT_FILENAME = "adi_robotarm.urdf";
    QString selectedFilter;

    QString fileName = QFileDialog::getSaveFileName( this,
                                                     "Save URDF",
                                                     DEFAULT_FILENAME,
                                                     "URDF files (*.urdf)",
                                                     &selectedFilter,
                                                     QFileDialog::DontUseNativeDialog
                                                    );
    if( !fileName.isEmpty() )
    {
        QFile xmlFile( fileName );

        if( xmlFile.open( QFile::WriteOnly | QFile::Text ) )
        {
            QTextStream xmlContent( &xmlFile );
            QDomDocument document;

            QByteArray xmlHeader = "<?xml version=\"1.0\"?>";
            document.setContent( xmlHeader );

            const QString COLOR_STR = "color";
            const QString MATERIAL_STR = "material";
            const QString NAME_STR = "name";
            const QString COLOR_RGBA = "rgba";

            QDomElement root = document.createElement( "robot" );
            root.setAttribute( NAME_STR, "visual" );
            document.appendChild( root );

            QDomComment comment = document.createComment( createCommentString( "Materials" ) );
            root.appendChild( comment );

            QDomElement materialBlue = document.createElement( MATERIAL_STR );
            materialBlue.setAttribute( NAME_STR, MATERIAL_NAMES.at( SerialRobot::MATERIAL_BLUE ) ) ;

            QDomElement colorBlue = document.createElement( COLOR_STR );
            colorBlue.setAttribute( COLOR_RGBA, createRgbaString( SerialRobot::COLOR_BLUE ) );
            materialBlue.appendChild( colorBlue );
            root.appendChild( materialBlue );


            QDomElement materialBlack = document.createElement( MATERIAL_STR );
            materialBlack.setAttribute( NAME_STR, MATERIAL_NAMES.at( SerialRobot::MATERIAL_BLACK ) ) ;

            QDomElement colorBlack = document.createElement( COLOR_STR );
            colorBlack.setAttribute( COLOR_RGBA, createRgbaString( SerialRobot::COLOR_BLACK ) );
            materialBlack.appendChild( colorBlack );
            root.appendChild( materialBlack );

            uint8_t robotPartIndex = 0;

            if( SerialRobot::PARTS_CHAIN_COUNT == mRobotPartsVec.size() )
            {
                const QString AXIS_STR = "axis";
                const QString CHILD_STR = "child";
                const QString COLLISION_STR = "collision";
                const QString EFFORT_STR = "effort";
                const QString GEOMETRY_STR = "geometry";
                const QString INERTIA_STR = "inertia";
                const QString INERTIAL_STR = "inertial";
                const QString I_XX_STR = "ixx";
                const QString I_XY_STR = "ixy";
                const QString I_XZ_STR = "ixz";
                const QString I_YY_STR = "iyy";
                const QString I_YZ_STR = "iyz";
                const QString I_ZZ_STR = "izz";
                const QString LENGTH_STR = "length";
                const QString LIMIT_STR = "limit";
                const QString LOWER_STR = "lower";
                const QString MASS_STR = "mass";
                const QString ORIGIN_STR = "origin";
                const QString PARENT_STR = "parent";
                const QString RADIUS_STR = "radius";
                const QString TYPE_STR = "type";
                const QString UPPER_STR = "upper";
                const QString VALUE_STR = "value";
                const QString VELOCITY_STR = "velocity";
                const QString VISUAL_STR = "visual";
                const QString XYZ_STR = "xyz";               


                //************************
                // 0. LINK - base
                //************************
                robotPartIndex = SerialRobot::PARTS_CHAIN_BASE_LINK;

                comment = document.createComment( createCommentString( mRobotPartsVec.at( robotPartIndex ) ) );
                root.appendChild( comment );

                QDomElement linkBaseLink = document.createElement( PART_NAMES.at( mRobotPartsVec.at( robotPartIndex ).partType ) );
                linkBaseLink.setAttribute( NAME_STR, QString::fromStdString( SerialRobot::PARTS_CHAIN_NAMES.at( static_cast<SerialRobot::PartsChain>( robotPartIndex ) ) ) );


                QDomElement baseLinkVisual = document.createElement( VISUAL_STR );

                QDomElement baseLinkVisualGeometry = document.createElement( GEOMETRY_STR );

                QDomElement baseLinkVisualGeometryCylinder = document.createElement( getVisualGeometry( mRobotPartsVec.at( robotPartIndex ) ) );
                baseLinkVisualGeometryCylinder.setAttribute( LENGTH_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.geometry.cylinder.length ) );
                baseLinkVisualGeometryCylinder.setAttribute( RADIUS_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.geometry.cylinder.radius ) );

                QDomElement baseLinkVisualOrigin = document.createElement( ORIGIN_STR );
                baseLinkVisualOrigin.setAttribute( XYZ_STR, getOriginCoordinates( mRobotPartsVec.at( robotPartIndex ) ) );

                QDomElement baseLinkVisualMaterial = document.createElement( MATERIAL_STR );
                baseLinkVisualMaterial.setAttribute( NAME_STR, getMaterialName( mRobotPartsVec.at( robotPartIndex ) ) );


                QDomElement baseLinkCollision = document.createElement( COLLISION_STR );

                QDomElement baseLinkCollisionGeometry = document.createElement( GEOMETRY_STR );

                QDomElement baseLinkCollisionGeometryCylinder = document.createElement( getVisualGeometry( mRobotPartsVec.at( robotPartIndex ) ) );
                baseLinkCollisionGeometryCylinder.setAttribute( LENGTH_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.geometry.cylinder.length ) );
                baseLinkCollisionGeometryCylinder.setAttribute( RADIUS_STR, QString::number( SerialRobot::ANTI_COLLISION_DIST + mRobotPartsVec.at( robotPartIndex ).link.geometry.cylinder.radius ) );

                QDomElement baseLinkCollisionOrigin = document.createElement( ORIGIN_STR );
                baseLinkCollisionOrigin.setAttribute( XYZ_STR, getOriginCoordinates( mRobotPartsVec.at( robotPartIndex ) ) );


                QDomElement baseLinkInertial = document.createElement( INERTIAL_STR );

                QDomElement baseLinkInertialMass = document.createElement( MASS_STR );
                baseLinkInertialMass.setAttribute( VALUE_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.mass ) );

                QDomElement baseLinkInertialOrigin = document.createElement( ORIGIN_STR );
                baseLinkInertialOrigin.setAttribute( XYZ_STR, getOriginCoordinates( mRobotPartsVec.at( robotPartIndex ) ) );

                QDomElement baseLinkInertialInertia = document.createElement( INERTIA_STR );
                baseLinkInertialInertia.setAttribute( I_XX_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.inertiaTensor[0][0] ) );
                baseLinkInertialInertia.setAttribute( I_XY_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.inertiaTensor[0][1] ) );
                baseLinkInertialInertia.setAttribute( I_XZ_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.inertiaTensor[0][2] ) );
                baseLinkInertialInertia.setAttribute( I_YY_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.inertiaTensor[1][1] ) );
                baseLinkInertialInertia.setAttribute( I_YZ_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.inertiaTensor[1][2] ) );
                baseLinkInertialInertia.setAttribute( I_ZZ_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.inertiaTensor[2][2] ) );


                baseLinkVisualGeometry.appendChild( baseLinkVisualGeometryCylinder );
                baseLinkVisual.appendChild( baseLinkVisualGeometry );
                baseLinkVisual.appendChild( baseLinkVisualOrigin );
                baseLinkVisual.appendChild( baseLinkVisualMaterial );
                linkBaseLink.appendChild( baseLinkVisual );

                baseLinkCollisionGeometry.appendChild( baseLinkCollisionGeometryCylinder );
                baseLinkCollision.appendChild( baseLinkCollisionGeometry );
                baseLinkCollision.appendChild( baseLinkCollisionOrigin );
                linkBaseLink.appendChild( baseLinkCollision );

                baseLinkInertial.appendChild( baseLinkInertialMass );
                baseLinkInertial.appendChild( baseLinkInertialOrigin );
                baseLinkInertial.appendChild( baseLinkInertialInertia );
                linkBaseLink.appendChild( baseLinkInertial );

                root.appendChild( linkBaseLink );


                //************************
                // 2. LINK - column
                //************************
                robotPartIndex = SerialRobot::PARTS_CHAIN_COLUMN_LINK;

                comment = document.createComment( createCommentString( mRobotPartsVec.at( robotPartIndex ) ) );
                root.appendChild( comment );

                QDomElement linkColumn = document.createElement( PART_NAMES.at( mRobotPartsVec.at( robotPartIndex ).partType ) );
                linkColumn.setAttribute( NAME_STR, QString::fromStdString( SerialRobot::PARTS_CHAIN_NAMES.at( static_cast<SerialRobot::PartsChain>( robotPartIndex ) ) ) );


                QDomElement columnVisual = document.createElement( VISUAL_STR );

                QDomElement columnVisualGeometry = document.createElement( GEOMETRY_STR );

                QDomElement columnVisualGeometryCylinder = document.createElement( getVisualGeometry( mRobotPartsVec.at( robotPartIndex ) ) );
                columnVisualGeometryCylinder.setAttribute( LENGTH_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.geometry.cylinder.length ) );
                columnVisualGeometryCylinder.setAttribute( RADIUS_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.geometry.cylinder.radius ) );

                QDomElement columnVisualOrigin = document.createElement( ORIGIN_STR );
                columnVisualOrigin.setAttribute( XYZ_STR, getOriginCoordinates( mRobotPartsVec.at( robotPartIndex ) ) );

                QDomElement columnVisualMaterial = document.createElement( MATERIAL_STR );
                columnVisualMaterial.setAttribute( NAME_STR, getMaterialName( mRobotPartsVec.at( robotPartIndex ) ) );


                QDomElement columnCollision = document.createElement( COLLISION_STR );

                QDomElement columnCollisionGeometry = document.createElement( GEOMETRY_STR );

                QDomElement columnCollisionGeometryCylinder = document.createElement( getVisualGeometry( mRobotPartsVec.at( robotPartIndex ) ) );
                columnCollisionGeometryCylinder.setAttribute( LENGTH_STR, QString::number( 0.8 * mRobotPartsVec.at( robotPartIndex ).link.geometry.cylinder.length ) );
                columnCollisionGeometryCylinder.setAttribute( RADIUS_STR, QString::number( SerialRobot::ANTI_COLLISION_DIST + mRobotPartsVec.at( robotPartIndex ).link.geometry.cylinder.radius ) );

                QDomElement columnCollisionOrigin = document.createElement( ORIGIN_STR );
                columnCollisionOrigin.setAttribute( XYZ_STR, getOriginCoordinates( mRobotPartsVec.at( robotPartIndex ) ) );


                QDomElement columnInertial = document.createElement( INERTIAL_STR );

                QDomElement columnInertialMass = document.createElement( MASS_STR );
                columnInertialMass.setAttribute( VALUE_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.mass ) );

                QDomElement columnInertialOrigin = document.createElement( ORIGIN_STR );
                columnInertialOrigin.setAttribute( XYZ_STR, getOriginCoordinates( mRobotPartsVec.at( robotPartIndex ) ) );

                QDomElement columnInertialInertia = document.createElement( INERTIA_STR );
                columnInertialInertia.setAttribute( I_XX_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.inertiaTensor[0][0] ) );
                columnInertialInertia.setAttribute( I_XY_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.inertiaTensor[0][1] ) );
                columnInertialInertia.setAttribute( I_XZ_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.inertiaTensor[0][2] ) );
                columnInertialInertia.setAttribute( I_YY_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.inertiaTensor[1][1] ) );
                columnInertialInertia.setAttribute( I_YZ_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.inertiaTensor[1][2] ) );
                columnInertialInertia.setAttribute( I_ZZ_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.inertiaTensor[2][2] ) );


                columnVisualGeometry.appendChild( columnVisualGeometryCylinder );
                columnVisual.appendChild( columnVisualGeometry );
                columnVisual.appendChild( columnVisualOrigin );
                columnVisual.appendChild( columnVisualMaterial );
                linkColumn.appendChild( columnVisual );

                columnCollisionGeometry.appendChild( columnCollisionGeometryCylinder );
                columnCollision.appendChild( columnCollisionGeometry );
                columnCollision.appendChild( columnCollisionOrigin );
                linkColumn.appendChild( columnCollision );

                columnInertial.appendChild( columnInertialMass );
                columnInertial.appendChild( columnInertialOrigin );
                columnInertial.appendChild( columnInertialInertia );
                linkColumn.appendChild( columnInertial );

                root.appendChild( linkColumn );


                //************************
                // 1. JOINT - base
                //************************
                robotPartIndex = SerialRobot::PARTS_CHAIN_BASE_JOINT;

                comment = document.createComment( createCommentString( mRobotPartsVec.at( robotPartIndex ) ) );
                root.appendChild( comment );

                QDomElement jointBase = document.createElement( PART_NAMES.at( mRobotPartsVec.at( robotPartIndex ).partType ) );
                jointBase.setAttribute( NAME_STR, QString::fromStdString( SerialRobot::PARTS_CHAIN_NAMES.at( static_cast<SerialRobot::PartsChain>( robotPartIndex ) ) ) );
                jointBase.setAttribute( TYPE_STR, getJointType( mRobotPartsVec.at( robotPartIndex ) ) );

                QDomElement jointBaseParent = document.createElement( PARENT_STR );
                jointBaseParent.setAttribute( PART_NAMES.at( SerialRobot::PART_LINK ),
                                              QString::fromStdString( SerialRobot::PARTS_CHAIN_NAMES.at( SerialRobot::PARTS_CHAIN_BASE_LINK ) ) );

                QDomElement jointBaseChild = document.createElement( CHILD_STR );
                jointBaseChild.setAttribute( PART_NAMES.at( SerialRobot::PART_LINK ),
                                             QString::fromStdString( SerialRobot::PARTS_CHAIN_NAMES.at( SerialRobot::PARTS_CHAIN_COLUMN_LINK ) ) );

                QDomElement jointBaseAxis = document.createElement( AXIS_STR );
                jointBaseAxis.setAttribute( XYZ_STR, getAxisString( mRobotPartsVec.at( robotPartIndex ) ) );

                QDomElement jointBaseOrigin = document.createElement( ORIGIN_STR );
                jointBaseOrigin.setAttribute( XYZ_STR, getOriginCoordinates( mRobotPartsVec.at( robotPartIndex ) ) );

                QDomElement jointBaseLimit = document.createElement( LIMIT_STR );
                jointBaseLimit.setAttribute( EFFORT_STR, mRobotPartsVec.at( robotPartIndex ).joint.limit.effort );

                if( SerialRobot::JOINT_TYPE_REVOLUTE == mRobotPartsVec.at( robotPartIndex ).joint.type )
                {
                    jointBaseLimit.setAttribute( LOWER_STR, mRobotPartsVec.at( robotPartIndex ).joint.limit.lowerAngleRad );
                    jointBaseLimit.setAttribute( UPPER_STR, mRobotPartsVec.at( robotPartIndex ).joint.limit.upperAngleRad );
                }

                jointBaseLimit.setAttribute( VELOCITY_STR, mRobotPartsVec.at( robotPartIndex ).joint.limit.velocity );

                jointBase.appendChild( jointBaseParent );
                jointBase.appendChild( jointBaseChild );
                jointBase.appendChild( jointBaseAxis );
                jointBase.appendChild( jointBaseOrigin );
                jointBase.appendChild( jointBaseLimit );

                root.appendChild( jointBase );


                //************************
                // 4. LINK - upper arm
                //************************
                robotPartIndex = SerialRobot::PARTS_CHAIN_UPPER_ARM_LINK;

                comment = document.createComment( createCommentString( mRobotPartsVec.at( robotPartIndex ) ) );
                root.appendChild( comment );

                QDomElement linkUpperArm = document.createElement( PART_NAMES.at( mRobotPartsVec.at( robotPartIndex ).partType ) );
                linkUpperArm.setAttribute( NAME_STR, QString::fromStdString( SerialRobot::PARTS_CHAIN_NAMES.at( static_cast<SerialRobot::PartsChain>( robotPartIndex ) ) ) );


                QDomElement upperArmVisual = document.createElement( VISUAL_STR );

                QDomElement upperArmVisualGeometry = document.createElement( GEOMETRY_STR );

                QDomElement upperArmVisualGeometryCylinder = document.createElement( getVisualGeometry( mRobotPartsVec.at( robotPartIndex ) ) );
                upperArmVisualGeometryCylinder.setAttribute( LENGTH_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.geometry.cylinder.length ) );
                upperArmVisualGeometryCylinder.setAttribute( RADIUS_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.geometry.cylinder.radius ) );

                QDomElement upperArmVisualOrigin = document.createElement( ORIGIN_STR );
                upperArmVisualOrigin.setAttribute( XYZ_STR, getOriginCoordinates( mRobotPartsVec.at( robotPartIndex ) ) );

                QDomElement upperArmVisualMaterial = document.createElement( MATERIAL_STR );
                upperArmVisualMaterial.setAttribute( NAME_STR, getMaterialName( mRobotPartsVec.at( robotPartIndex ) ) );


                QDomElement upperArmCollision = document.createElement( COLLISION_STR );

                QDomElement upperArmCollisionGeometry = document.createElement( GEOMETRY_STR );

                QDomElement upperArmCollisionGeometryCylinder = document.createElement( getVisualGeometry( mRobotPartsVec.at( robotPartIndex ) ) );
                upperArmCollisionGeometryCylinder.setAttribute( LENGTH_STR, QString::number( 0.8 * mRobotPartsVec.at( robotPartIndex ).link.geometry.cylinder.length ) );
                upperArmCollisionGeometryCylinder.setAttribute( RADIUS_STR, QString::number( SerialRobot::ANTI_COLLISION_DIST + mRobotPartsVec.at( robotPartIndex ).link.geometry.cylinder.radius ) );

                QDomElement upperArmCollisionOrigin = document.createElement( ORIGIN_STR );
                upperArmCollisionOrigin.setAttribute( XYZ_STR, getOriginCoordinates( mRobotPartsVec.at( robotPartIndex ) ) );


                QDomElement upperArmInertial = document.createElement( INERTIAL_STR );

                QDomElement upperArmInertialMass = document.createElement( MASS_STR );
                upperArmInertialMass.setAttribute( VALUE_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.mass ) );

                QDomElement upperArmInertialOrigin = document.createElement( ORIGIN_STR );
                upperArmInertialOrigin.setAttribute( XYZ_STR, getOriginCoordinates( mRobotPartsVec.at( robotPartIndex ) ) );

                QDomElement upperArmInertialInertia = document.createElement( INERTIA_STR );
                upperArmInertialInertia.setAttribute( I_XX_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.inertiaTensor[0][0] ) );
                upperArmInertialInertia.setAttribute( I_XY_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.inertiaTensor[0][1] ) );
                upperArmInertialInertia.setAttribute( I_XZ_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.inertiaTensor[0][2] ) );
                upperArmInertialInertia.setAttribute( I_YY_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.inertiaTensor[1][1] ) );
                upperArmInertialInertia.setAttribute( I_YZ_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.inertiaTensor[1][2] ) );
                upperArmInertialInertia.setAttribute( I_ZZ_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.inertiaTensor[2][2] ) );


                upperArmVisualGeometry.appendChild( upperArmVisualGeometryCylinder );
                upperArmVisual.appendChild( upperArmVisualGeometry );
                upperArmVisual.appendChild( upperArmVisualOrigin );
                upperArmVisual.appendChild( upperArmVisualMaterial );
                linkUpperArm.appendChild( upperArmVisual );

                upperArmCollisionGeometry.appendChild( upperArmCollisionGeometryCylinder );
                upperArmCollision.appendChild( upperArmCollisionGeometry );
                upperArmCollision.appendChild( upperArmCollisionOrigin );
                linkUpperArm.appendChild( upperArmCollision );

                upperArmInertial.appendChild( upperArmInertialMass );
                upperArmInertial.appendChild( upperArmInertialOrigin );
                upperArmInertial.appendChild( upperArmInertialInertia );
                linkUpperArm.appendChild( upperArmInertial );

                root.appendChild( linkUpperArm );


                //************************
                // 3. JOINT - shoulder
                //************************
                robotPartIndex = SerialRobot::PARTS_CHAIN_SHOULDER_JOINT;

                comment = document.createComment( createCommentString( mRobotPartsVec.at( robotPartIndex ) ) );
                root.appendChild( comment );

                QDomElement jointShoulder = document.createElement( PART_NAMES.at( mRobotPartsVec.at( robotPartIndex ).partType ) );
                jointShoulder.setAttribute( NAME_STR, QString::fromStdString( SerialRobot::PARTS_CHAIN_NAMES.at( static_cast<SerialRobot::PartsChain>( robotPartIndex ) ) ) );
                jointShoulder.setAttribute( TYPE_STR, getJointType( mRobotPartsVec.at( robotPartIndex ) ) );

                QDomElement jointShoulderParent = document.createElement( PARENT_STR );
                jointShoulderParent.setAttribute( PART_NAMES.at( SerialRobot::PART_LINK ),
                                                  QString::fromStdString( SerialRobot::PARTS_CHAIN_NAMES.at( SerialRobot::PARTS_CHAIN_COLUMN_LINK ) ) );

                QDomElement jointShoulderChild = document.createElement( CHILD_STR );
                jointShoulderChild.setAttribute( PART_NAMES.at( SerialRobot::PART_LINK ),
                                                 QString::fromStdString( SerialRobot::PARTS_CHAIN_NAMES.at( SerialRobot::PARTS_CHAIN_UPPER_ARM_LINK ) ) );

                QDomElement jointShoulderAxis = document.createElement( AXIS_STR );
                jointShoulderAxis.setAttribute( XYZ_STR, getAxisString( mRobotPartsVec.at( robotPartIndex ) ) );

                QDomElement jointShoulderOrigin = document.createElement( ORIGIN_STR );
                jointShoulderOrigin.setAttribute( XYZ_STR, getOriginCoordinates( mRobotPartsVec.at( robotPartIndex ) ) );

                QDomElement jointShoulderLimit = document.createElement( LIMIT_STR );
                jointShoulderLimit.setAttribute( EFFORT_STR, mRobotPartsVec.at( robotPartIndex ).joint.limit.effort );
                jointShoulderLimit.setAttribute( LOWER_STR, mRobotPartsVec.at( robotPartIndex ).joint.limit.lowerAngleRad );
                jointShoulderLimit.setAttribute( UPPER_STR, mRobotPartsVec.at( robotPartIndex ).joint.limit.upperAngleRad );
                jointShoulderLimit.setAttribute( VELOCITY_STR, mRobotPartsVec.at( robotPartIndex ).joint.limit.velocity );

                jointShoulder.appendChild( jointShoulderParent );
                jointShoulder.appendChild( jointShoulderChild );
                jointShoulder.appendChild( jointShoulderAxis );
                jointShoulder.appendChild( jointShoulderOrigin );
                jointShoulder.appendChild( jointShoulderLimit );

                root.appendChild( jointShoulder );


                //************************
                // 6. LINK - lower arm
                //************************
                robotPartIndex = SerialRobot::PARTS_CHAIN_LOWER_ARM_LINK;

                comment = document.createComment( createCommentString( mRobotPartsVec.at( robotPartIndex ) ) );
                root.appendChild( comment );

                QDomElement linkLowerArm = document.createElement( PART_NAMES.at( mRobotPartsVec.at( robotPartIndex ).partType ) );
                linkLowerArm.setAttribute( NAME_STR, QString::fromStdString( SerialRobot::PARTS_CHAIN_NAMES.at( static_cast<SerialRobot::PartsChain>( robotPartIndex ) ) ) );


                QDomElement lowerArmVisual = document.createElement( VISUAL_STR );

                QDomElement lowerArmVisualGeometry = document.createElement( GEOMETRY_STR );

                QDomElement lowerArmVisualGeometryCylinder = document.createElement( getVisualGeometry( mRobotPartsVec.at( robotPartIndex ) ) );
                lowerArmVisualGeometryCylinder.setAttribute( LENGTH_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.geometry.cylinder.length ) );
                lowerArmVisualGeometryCylinder.setAttribute( RADIUS_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.geometry.cylinder.radius ) );

                QDomElement lowerArmVisualOrigin = document.createElement( ORIGIN_STR );
                lowerArmVisualOrigin.setAttribute( XYZ_STR, getOriginCoordinates( mRobotPartsVec.at( robotPartIndex ) ) );

                QDomElement lowerArmVisualMaterial = document.createElement( MATERIAL_STR );
                lowerArmVisualMaterial.setAttribute( NAME_STR, getMaterialName( mRobotPartsVec.at( robotPartIndex ) ) );


                QDomElement lowerArmCollision = document.createElement( COLLISION_STR );

                QDomElement lowerArmCollisionGeometry = document.createElement( GEOMETRY_STR );

                QDomElement lowerArmCollisionGeometryCylinder = document.createElement( getVisualGeometry( mRobotPartsVec.at( robotPartIndex ) ) );
                lowerArmCollisionGeometryCylinder.setAttribute( LENGTH_STR, QString::number( 0.8 * mRobotPartsVec.at( robotPartIndex ).link.geometry.cylinder.length ) );
                lowerArmCollisionGeometryCylinder.setAttribute( RADIUS_STR, QString::number( SerialRobot::ANTI_COLLISION_DIST + mRobotPartsVec.at( robotPartIndex ).link.geometry.cylinder.radius ) );

                QDomElement lowerArmCollisionOrigin = document.createElement( ORIGIN_STR );
                lowerArmCollisionOrigin.setAttribute( XYZ_STR, getOriginCoordinates( mRobotPartsVec.at( robotPartIndex ) ) );


                QDomElement lowerArmInertial = document.createElement( INERTIAL_STR );

                QDomElement lowerArmInertialMass = document.createElement( MASS_STR );
                lowerArmInertialMass.setAttribute( VALUE_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.mass ) );

                QDomElement lowerArmInertialOrigin = document.createElement( ORIGIN_STR );
                lowerArmInertialOrigin.setAttribute( XYZ_STR, getOriginCoordinates( mRobotPartsVec.at( robotPartIndex ) ) );

                QDomElement lowerArmInertialInertia = document.createElement( INERTIA_STR );
                lowerArmInertialInertia.setAttribute( I_XX_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.inertiaTensor[0][0] ) );
                lowerArmInertialInertia.setAttribute( I_XY_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.inertiaTensor[0][1] ) );
                lowerArmInertialInertia.setAttribute( I_XZ_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.inertiaTensor[0][2] ) );
                lowerArmInertialInertia.setAttribute( I_YY_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.inertiaTensor[1][1] ) );
                lowerArmInertialInertia.setAttribute( I_YZ_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.inertiaTensor[1][2] ) );
                lowerArmInertialInertia.setAttribute( I_ZZ_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.inertiaTensor[2][2] ) );


                lowerArmVisualGeometry.appendChild( lowerArmVisualGeometryCylinder );
                lowerArmVisual.appendChild( lowerArmVisualGeometry );
                lowerArmVisual.appendChild( lowerArmVisualOrigin );
                lowerArmVisual.appendChild( lowerArmVisualMaterial );
                linkLowerArm.appendChild( lowerArmVisual );

                lowerArmCollisionGeometry.appendChild( lowerArmCollisionGeometryCylinder );
                lowerArmCollision.appendChild( lowerArmCollisionGeometry );
                lowerArmCollision.appendChild( lowerArmCollisionOrigin );
                linkLowerArm.appendChild( lowerArmCollision );

                lowerArmInertial.appendChild( lowerArmInertialMass );
                lowerArmInertial.appendChild( lowerArmInertialOrigin );
                lowerArmInertial.appendChild( lowerArmInertialInertia );
                linkLowerArm.appendChild( lowerArmInertial );

                root.appendChild( linkLowerArm );


                //************************
                // 5. JOINT - elbow
                //************************
                robotPartIndex = SerialRobot::PARTS_CHAIN_ELBOW_JOINT;

                comment = document.createComment( createCommentString( mRobotPartsVec.at( robotPartIndex ) ) );
                root.appendChild( comment );

                QDomElement jointElbow = document.createElement( PART_NAMES.at( mRobotPartsVec.at( robotPartIndex ).partType ) );
                jointElbow.setAttribute( NAME_STR, QString::fromStdString( SerialRobot::PARTS_CHAIN_NAMES.at( static_cast<SerialRobot::PartsChain>( robotPartIndex ) ) ) );
                jointElbow.setAttribute( TYPE_STR, getJointType( mRobotPartsVec.at( robotPartIndex ) ) );

                QDomElement jointElbowParent = document.createElement( PARENT_STR );
                jointElbowParent.setAttribute( PART_NAMES.at( SerialRobot::PART_LINK ),
                                               QString::fromStdString( SerialRobot::PARTS_CHAIN_NAMES.at( SerialRobot::PARTS_CHAIN_UPPER_ARM_LINK ) ) );

                QDomElement jointElbowChild = document.createElement( CHILD_STR );
                jointElbowChild.setAttribute( PART_NAMES.at( SerialRobot::PART_LINK ),
                                              QString::fromStdString( SerialRobot::PARTS_CHAIN_NAMES.at( SerialRobot::PARTS_CHAIN_LOWER_ARM_LINK ) ) );

                QDomElement jointElbowAxis = document.createElement( AXIS_STR );
                jointElbowAxis.setAttribute( XYZ_STR, getAxisString( mRobotPartsVec.at( robotPartIndex ) ) );

                QDomElement jointElbowOrigin = document.createElement( ORIGIN_STR );
                jointElbowOrigin.setAttribute( XYZ_STR, getOriginCoordinates( mRobotPartsVec.at( robotPartIndex ) ) );

                QDomElement jointElbowLimit = document.createElement( LIMIT_STR );
                jointElbowLimit.setAttribute( EFFORT_STR, mRobotPartsVec.at( robotPartIndex ).joint.limit.effort );
                jointElbowLimit.setAttribute( LOWER_STR, mRobotPartsVec.at( robotPartIndex ).joint.limit.lowerAngleRad );
                jointElbowLimit.setAttribute( UPPER_STR, mRobotPartsVec.at( robotPartIndex ).joint.limit.upperAngleRad );
                jointElbowLimit.setAttribute( VELOCITY_STR, mRobotPartsVec.at( robotPartIndex ).joint.limit.velocity );

                jointElbow.appendChild( jointElbowParent );
                jointElbow.appendChild( jointElbowChild );
                jointElbow.appendChild( jointElbowAxis );
                jointElbow.appendChild( jointElbowOrigin );
                jointElbow.appendChild( jointElbowLimit );

                root.appendChild( jointElbow );


                //************************
                // 8. LINK - end effector
                //************************
                robotPartIndex = SerialRobot::PARTS_CHAIN_END_EFFECTOR_LINK;

                comment = document.createComment( createCommentString( mRobotPartsVec.at( robotPartIndex ) ) );
                root.appendChild( comment );

                QDomElement linkEndEffector = document.createElement( PART_NAMES.at( mRobotPartsVec.at( robotPartIndex ).partType ) );
                linkEndEffector.setAttribute( NAME_STR, QString::fromStdString( SerialRobot::PARTS_CHAIN_NAMES.at( static_cast<SerialRobot::PartsChain>( robotPartIndex ) ) ) );


                QDomElement endEffectorVisual = document.createElement( VISUAL_STR );

                QDomElement endEffectorVisualGeometry = document.createElement( GEOMETRY_STR );

                QDomElement endEffectorVisualGeometryCylinder = document.createElement( getVisualGeometry( mRobotPartsVec.at( robotPartIndex ) ) );
                endEffectorVisualGeometryCylinder.setAttribute( LENGTH_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.geometry.cylinder.length ) );
                endEffectorVisualGeometryCylinder.setAttribute( RADIUS_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.geometry.cylinder.radius ) );

                QDomElement endEffectorVisualOrigin = document.createElement( ORIGIN_STR );
                endEffectorVisualOrigin.setAttribute( XYZ_STR, getOriginCoordinates( mRobotPartsVec.at( robotPartIndex ) ) );

                QDomElement endEffectorVisualMaterial = document.createElement( MATERIAL_STR );
                endEffectorVisualMaterial.setAttribute( NAME_STR, getMaterialName( mRobotPartsVec.at( robotPartIndex ) ) );


                QDomElement endEffectorCollision = document.createElement( COLLISION_STR );

                QDomElement endEffectorCollisionGeometry = document.createElement( GEOMETRY_STR );

                QDomElement endEffectorCollisionGeometryCylinder = document.createElement( getVisualGeometry( mRobotPartsVec.at( robotPartIndex ) ) );
                endEffectorCollisionGeometryCylinder.setAttribute( LENGTH_STR, QString::number( 0.8 * mRobotPartsVec.at( robotPartIndex ).link.geometry.cylinder.length ) );
                endEffectorCollisionGeometryCylinder.setAttribute( RADIUS_STR, QString::number( SerialRobot::ANTI_COLLISION_DIST + mRobotPartsVec.at( robotPartIndex ).link.geometry.cylinder.radius ) );

                QDomElement endEffectorCollisionOrigin = document.createElement( ORIGIN_STR );
                endEffectorCollisionOrigin.setAttribute( XYZ_STR, getOriginCoordinates( mRobotPartsVec.at( robotPartIndex ) ) );


                QDomElement endEffectorInertial = document.createElement( INERTIAL_STR );

                QDomElement endEffectorInertialMass = document.createElement( MASS_STR );
                endEffectorInertialMass.setAttribute( VALUE_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.mass ) );

                QDomElement endEffectorInertialOrigin = document.createElement( ORIGIN_STR );
                endEffectorInertialOrigin.setAttribute( XYZ_STR, getOriginCoordinates( mRobotPartsVec.at( robotPartIndex ) ) );

                QDomElement endEffectorInertialInertia = document.createElement( INERTIA_STR );
                endEffectorInertialInertia.setAttribute( I_XX_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.inertiaTensor[0][0] ) );
                endEffectorInertialInertia.setAttribute( I_XY_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.inertiaTensor[0][1] ) );
                endEffectorInertialInertia.setAttribute( I_XZ_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.inertiaTensor[0][2] ) );
                endEffectorInertialInertia.setAttribute( I_YY_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.inertiaTensor[1][1] ) );
                endEffectorInertialInertia.setAttribute( I_YZ_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.inertiaTensor[1][2] ) );
                endEffectorInertialInertia.setAttribute( I_ZZ_STR, QString::number( mRobotPartsVec.at( robotPartIndex ).link.inertiaTensor[2][2] ) );


                endEffectorVisualGeometry.appendChild( endEffectorVisualGeometryCylinder );
                endEffectorVisual.appendChild( endEffectorVisualGeometry );
                endEffectorVisual.appendChild( endEffectorVisualOrigin );
                endEffectorVisual.appendChild( endEffectorVisualMaterial );
                linkEndEffector.appendChild( endEffectorVisual );

                endEffectorCollisionGeometry.appendChild( endEffectorCollisionGeometryCylinder );
                endEffectorCollision.appendChild( endEffectorCollisionGeometry );
                endEffectorCollision.appendChild( endEffectorCollisionOrigin );
                linkEndEffector.appendChild( endEffectorCollision );

                endEffectorInertial.appendChild( endEffectorInertialMass );
                endEffectorInertial.appendChild( endEffectorInertialOrigin );
                endEffectorInertial.appendChild( endEffectorInertialInertia );
                linkEndEffector.appendChild( endEffectorInertial );

                root.appendChild( linkEndEffector );


                //************************
                // 7. JOINT - wrist
                //************************
                robotPartIndex = SerialRobot::PARTS_CHAIN_WRIST_JOINT;

                comment = document.createComment( createCommentString( mRobotPartsVec.at( robotPartIndex ) ) );
                root.appendChild( comment );

                QDomElement jointWrist = document.createElement( PART_NAMES.at( mRobotPartsVec.at( robotPartIndex ).partType ) );
                jointWrist.setAttribute( NAME_STR, QString::fromStdString( SerialRobot::PARTS_CHAIN_NAMES.at( static_cast<SerialRobot::PartsChain>( robotPartIndex ) ) ) );
                jointWrist.setAttribute( TYPE_STR, getJointType( mRobotPartsVec.at( robotPartIndex ) ) );

                QDomElement jointWristParent = document.createElement( PARENT_STR );
                jointWristParent.setAttribute( PART_NAMES.at( SerialRobot::PART_LINK ),
                                               QString::fromStdString( SerialRobot::PARTS_CHAIN_NAMES.at( SerialRobot::PARTS_CHAIN_LOWER_ARM_LINK ) ) );

                QDomElement jointWristChild = document.createElement( CHILD_STR );
                jointWristChild.setAttribute( PART_NAMES.at( SerialRobot::PART_LINK ),
                                              QString::fromStdString( SerialRobot::PARTS_CHAIN_NAMES.at( SerialRobot::PARTS_CHAIN_END_EFFECTOR_LINK ) ) );

                QDomElement jointWristAxis = document.createElement( AXIS_STR );
                jointWristAxis.setAttribute( XYZ_STR, getAxisString( mRobotPartsVec.at( robotPartIndex ) ) );

                QDomElement jointWristOrigin = document.createElement( ORIGIN_STR );
                jointWristOrigin.setAttribute( XYZ_STR, getOriginCoordinates( mRobotPartsVec.at( robotPartIndex ) ) );

                QDomElement jointWristLimit = document.createElement( LIMIT_STR );
                jointWristLimit.setAttribute( EFFORT_STR, mRobotPartsVec.at( robotPartIndex ).joint.limit.effort );
                jointWristLimit.setAttribute( LOWER_STR, mRobotPartsVec.at( robotPartIndex ).joint.limit.lowerAngleRad );
                jointWristLimit.setAttribute( UPPER_STR, mRobotPartsVec.at( robotPartIndex ).joint.limit.upperAngleRad );
                jointWristLimit.setAttribute( VELOCITY_STR, mRobotPartsVec.at( robotPartIndex ).joint.limit.velocity );

                jointWrist.appendChild( jointWristParent );
                jointWrist.appendChild( jointWristChild );
                jointWrist.appendChild( jointWristAxis );
                jointWrist.appendChild( jointWristOrigin );
                jointWrist.appendChild( jointWristLimit );

                root.appendChild( jointWrist );
            }

            xmlContent << document.toString();
            xmlFile.close();
        }
        else
        {
            QMessageBox::critical( this, "Robot Arm Kinematic",
                                   "\nCannot open the file for writing.",
                                   QMessageBox::Ok );
        }
    }
}


//!************************************************************************
//! Get the string with orientation axes for a robot part
//!
//! @returns The string with orientation axes
//!************************************************************************
QString UrdfHandler::getAxisString
    (
    const SerialRobot::RobotPart aRobotPart //!< robot part
    ) const
{
    QString axisString;

    if( SerialRobot::PART_JOINT == aRobotPart.partType )
    {
        SerialRobot::TriaxialOrientation axis = aRobotPart.joint.orientation;
        axisString += QString::number( axis.xAxis );
        axisString += " ";
        axisString += QString::number( axis.yAxis );
        axisString += " ";
        axisString += QString::number( axis.zAxis );
    }

    return axisString;
}


//!************************************************************************
//! Get the string with box dimensions for a robot part
//!
//! @returns The string with box dimensions
//!************************************************************************
QString UrdfHandler::getBoxDimensions
    (
    const SerialRobot::RobotPart aRobotPart //!< robot part
    ) const
{
    QString boxDimensions;

    if( SerialRobot::PART_LINK == aRobotPart.partType
     && SerialRobot::GEOMETRIC_SHAPE_PARALLELEPIPED == aRobotPart.link.geometricShape )
    {
        boxDimensions += QString::number( aRobotPart.link.geometry.box.xSize );
        boxDimensions += " ";
        boxDimensions += QString::number( aRobotPart.link.geometry.box.ySize );
        boxDimensions += " ";
        boxDimensions += QString::number( aRobotPart.link.geometry.box.zSize );
    }

    return boxDimensions;
}


//!************************************************************************
//! Get the string with joint type for a robot part
//!
//! @returns The string with joint type
//!************************************************************************
QString UrdfHandler::getJointType
    (
    const SerialRobot::RobotPart aRobotPart //!< robot part
    ) const
{
    QString jointType;

    if( SerialRobot::PART_JOINT == aRobotPart.partType )
    {
        jointType = JOINT_TYPE_NAMES.at( aRobotPart.joint.type );
    }

    return jointType;
}


//!************************************************************************
//! Get the material name for a robot part
//!
//! @returns The string with material name
//!************************************************************************
QString UrdfHandler::getMaterialName
    (
    const SerialRobot::RobotPart aRobotPart //!< robot part
    ) const
{
    QString materialName;

    if( SerialRobot::PART_LINK == aRobotPart.partType )
    {
        materialName = MATERIAL_NAMES.at( aRobotPart.link.material );
    }

    return materialName;
}


//!************************************************************************
//! Get the string with origin coordinates for a robot part
//!
//! @returns The string with origin coordinates
//!************************************************************************
QString UrdfHandler::getOriginCoordinates
    (
    const SerialRobot::RobotPart aRobotPart //!< robot part
    ) const
{
    QString originCoords;
    SerialRobot::TriaxialPoint origin;

    switch( aRobotPart.partType )
    {
        case SerialRobot::PART_LINK:
            origin = aRobotPart.link.origin;
            break;

        case SerialRobot::PART_JOINT:
            origin = aRobotPart.joint.origin;
            break;

        default:
            break;
    }

    originCoords += QString::number( origin.x );
    originCoords += " ";
    originCoords += QString::number( origin.y );
    originCoords += " ";
    originCoords += QString::number( origin.z );

    return originCoords;
}


//!************************************************************************
//! Get the string with visual geometry for a robot part
//!
//! @returns The string with visual geometry
//!************************************************************************
QString UrdfHandler::getVisualGeometry
    (
    const SerialRobot::RobotPart aRobotPart //!< robot part
    ) const
{
    QString visualGeometry;

    if( SerialRobot::PART_LINK == aRobotPart.partType )
    {
        visualGeometry = GEOMETRIC_SHAPE_NAMES.at( aRobotPart.link.geometricShape );
    }

    return visualGeometry;
}


//!************************************************************************
//! Set the vector with robot parts
//!
//! @returns nothing
//!************************************************************************
void UrdfHandler::setRobotPartsVec
    (
    const std::vector<SerialRobot::RobotPart>   aRobotPartsVec      //!< robot parts vector
    )
{
    mRobotPartsVec = aRobotPartsVec;
}
