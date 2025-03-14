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
UrdfHandler.h

This file contains the definitions for the URDF handler.
*/

#ifndef UrdfHandler_h
#define UrdfHandler_h

#include "SerialRobot.h"

#include <QString>
#include <QWidget>

#include <cstdint>
#include <map>
#include <string>
#include <vector>


//************************************************************************
// Class for handling URDF files
//************************************************************************
class UrdfHandler : public QWidget
{
    Q_OBJECT

    //************************************************************************
    // constants and types
    //************************************************************************
    private:
        static const uint8_t COMMENT_LINE_LEN = 30;

        static const std::map<SerialRobot::Part, QString> PART_NAMES;

        static const std::map<SerialRobot::JointType, QString> JOINT_TYPE_NAMES;

        static const std::map<SerialRobot::GeometricShape, QString> GEOMETRIC_SHAPE_NAMES;

        static const std::map<SerialRobot::Material, QString> MATERIAL_NAMES;


    //************************************************************************
    // functions
    //************************************************************************
    public:
        UrdfHandler();   

        void generateFile();

        void setRobotPartsVec
            (
            const std::vector<SerialRobot::RobotPart>   aRobotPartsVec      //!< robot parts vector
            );

    private:
        QString createCommentString
            (
            const std::string aString                   //!< comment string
            ) const;

        QString createCommentString
            (
            const SerialRobot::RobotPart aRobotPart //!< robot part
            ) const;

        QString createRgbaString
            (
            const SerialRobot::ColorRgba aColor     //!< RGBA color
            ) const;

        QString getAxisString
            (
            const SerialRobot::RobotPart aRobotPart //!< robot part
            ) const;

        QString getBoxDimensions
            (
            const SerialRobot::RobotPart aRobotPart //!< robot part
            ) const;

        QString getJointType
            (
            const SerialRobot::RobotPart aRobotPart //!< robot part
            ) const;

        QString getMaterialName
            (
            const SerialRobot::RobotPart aRobotPart //!< robot part
            ) const;

        QString getOriginCoordinates
            (
            const SerialRobot::RobotPart aRobotPart //!< robot part
            ) const;

        QString getVisualGeometry
            (
            const SerialRobot::RobotPart aRobotPart //!< robot part
            ) const;


    //************************************************************************
    // variables
    //************************************************************************
    private:
        std::vector<SerialRobot::RobotPart>             mRobotPartsVec;     //!< robot parts vector
};

#endif // UrdfHandler_h
