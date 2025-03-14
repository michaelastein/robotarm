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
XrdfHandler.h

This file contains the definitions for the Nvidia XRDF handler.
*/

#ifndef XrdfHandler_h
#define XrdfHandler_h

#include "SerialRobot.h"

#include <QString>
#include <QWidget>

#include <cstdint>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>


//************************************************************************
// Class for handling Nvidia XRDF files
//************************************************************************
class XrdfHandler : public QWidget
{
    Q_OBJECT

    //************************************************************************
    // functions
    //************************************************************************
    public:
        XrdfHandler();

        void generateFile();

        void setRobotPartsVec
            (
            const std::vector<SerialRobot::RobotPart>   aRobotPartsVec      //!< robot parts vector
            );

    private:
        QString customizeCoordinateStr
            (
            const double aCoord     //!< coordinate
            ) const;

        std::string getBaseFrame() const;

        std::string getGeometryName() const;

        std::string indent
            (
            const uint8_t aLevel    //!< level of indent
            ) const;

        void writeCollision();

        void writeCspace();

        void writeDefaultJointPositions();

        void writeFormatVersion();

        void writeGeometry();

        void writeModifiers();

        void writeSelfCollision();

        void writeToolFrames();


    //************************************************************************
    // variables
    //************************************************************************
    private:
        std::ofstream                           mOutputFile;        //!< output file
        std::vector<SerialRobot::RobotPart>     mRobotPartsVec;     //!< robot parts vector
};

#endif // XrdfHandler_h
