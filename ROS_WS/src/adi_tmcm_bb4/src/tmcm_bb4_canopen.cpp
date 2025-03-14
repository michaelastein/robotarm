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
tmcm_bb4_canopen.cpp

This file contains the sources for the ROS2 application of TMCM-BB4 CANopen motion control.
*/

#include <adi_tmcm_bb4/TmcmBb4CanOpen.hpp>

#include <cstdio>
#include <memory>
#include <string>

#include <QCoreApplication>


//!************************************************************************
//! Main application
//!
//! @returns: execution code
//!************************************************************************
int main
    (
    int     argc,   //!< argument count
    char*   argv[]  //!< argument vector
    )
{
    QCoreApplication a( argc, argv );
    setvbuf( stdout, NULL, _IONBF, BUFSIZ );
    rclcpp::init( argc, argv );
    bool resetHomePosition = false;

    if( 2 == argc )
    {
        std::string arg1 = argv[1];
        resetHomePosition = ( "rhp" == arg1 );
    }

    auto tmcmBb4 = std::make_shared<TmcmBb4CanOpen>( resetHomePosition );
    rclcpp::spin( tmcmBb4 );

    rclcpp::shutdown();
    return a.exec();
}
