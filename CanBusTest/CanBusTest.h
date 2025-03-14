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
CanBusTest.h

This file contains the definitions for testing CAN bus communication.
*/

#ifndef CanBusTest_h
#define CanBusTest_h

#include "CanOpen.h"

#include <QMainWindow>
#include <QString>

#include <cstdint>
#include <vector>

QT_BEGIN_NAMESPACE
    namespace Ui
    {
        class CanBusTest;
    }
QT_END_NAMESPACE


//************************************************************************
// Class for handling the TMCM-BB4 board
//************************************************************************
class CanBusTest : public QMainWindow
{
    Q_OBJECT

    //************************************************************************
    // constants and types
    //************************************************************************
    private:
        typedef enum : uint8_t
        {
            MODE_TX,
            MODE_RX
        }Mode;


    //************************************************************************
    // functions
    //************************************************************************
    public:
        CanBusTest
            (
            QWidget*    aParent = nullptr   //!< parent widget
            );

        ~CanBusTest();

    private:
        uint8_t getRandomByte() const;

    private slots:
        void handleCanConnect();

        void handleCanDisconnect();

        void handleCanReset();

        void handleExit();

        void handleFramesCountChanged
            (
            int aValue  //!< number of frames
            );

        void handleLoopbackChanged
            (
            bool aStatus            //!< true if checked
            );

        void handleStart();

        void updateCanConnectStatus
            (
            const bool aStatus      //!< true if connected
            );

        void updateCanConnectionStr
            (
            const QString aString   //!< string
            );

        void updateCanStatusStr
            (
            const QString aString   //!< string
            );

        void updateMode
            (
            int aIndex   //!< operating mode index
            );


    //************************************************************************
    // variables
    //************************************************************************
    private:
        Ui::CanBusTest*                 mMainUi;            //!< main UI

        CanOpen*                        mCanOpen;           //!< CANopen instance
        bool                            mLoopbackActive;    //!< if using CAN loopback
        Mode                            mMode;              //!< mode of operation
        uint32_t                        mTestFramesCount;   //!< number of test frames
};

#endif // CanBusTest_h
