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
TmcmBb4.h

This file contains the definitions for TMCM-BB4.
*/

#ifndef TmcmBb4_h
#define TmcmBb4_h

#include "CanOpen.h"
#include "Tmcm1637CanOpen.h"

#include <QLabel>
#include <QMainWindow>
#include <QString>
#include <QTimer>

#include <cstdint>
#include <vector>

QT_BEGIN_NAMESPACE
    namespace Ui
    {
        class TmcmBb4;
    }
QT_END_NAMESPACE


//************************************************************************
// Class for handling the TMCM-BB4 board
//************************************************************************
class TmcmBb4 : public QMainWindow
{
    Q_OBJECT

    //************************************************************************
    // constants and types
    //************************************************************************
    private:
        static const uint8_t TMCM1637_SLOTS = 4;    //!< number of motion controller slots

        const QString NA_STR = "N/A";
        const QString ENABLE_STR = "Enable";
        const QString DISABLE_STR = "Disable";


    //************************************************************************
    // functions
    //************************************************************************
    public:
        TmcmBb4
            (
            QWidget*    aParent = nullptr   //!< parent widget
            );

        ~TmcmBb4();

    private:
        void checkMotionControllers();

        QString formatHexString
            (
            const uint32_t  aValue,     //!< value to convert
            const uint32_t  aMaxValue   //!< maximum expected value
            ) const;

        void updateControlsForOperatingMode0
            (
            const Tmcm1637CanOpen::OperatingMode aOperMode //!< operating mode
            );
        void updateControlsForOperatingMode1
            (
            const Tmcm1637CanOpen::OperatingMode aOperMode //!< operating mode
            );
        void updateControlsForOperatingMode2
            (
            const Tmcm1637CanOpen::OperatingMode aOperMode //!< operating mode
            );
        void updateControlsForOperatingMode3
            (
            const Tmcm1637CanOpen::OperatingMode aOperMode //!< operating mode
            );

    private slots:
        void handleCanConnect();

        void handleCanDisconnect();

        void handleCanReset();

        void handleExit();


        void handleHomeUpdate0();
        void handleHomeUpdate1();
        void handleHomeUpdate2();
        void handleHomeUpdate3();

        void handleOperatingMode0
            (
            int aIndex  //!< index
            );
        void handleOperatingMode1
            (
            int aIndex  //!< index
            );
        void handleOperatingMode2
            (
            int aIndex  //!< index
            );
        void handleOperatingMode3
            (
            int aIndex  //!< index
            );

        void handleOperationEnable0();
        void handleOperationEnable1();
        void handleOperationEnable2();
        void handleOperationEnable3();

        void handlePosition0
            (
            int aValue  //!< value
            );
        void handlePosition1
            (
            int aValue  //!< value
            );
        void handlePosition2
            (
            int aValue  //!< value
            );
        void handlePosition3
            (
            int aValue  //!< value
            );

        void handleVelocity0
            (
            int aValue  //!< value
            );
        void handleVelocity1
            (
            int aValue  //!< value
            );
        void handleVelocity2
            (
            int aValue  //!< value
            );
        void handleVelocity3
            (
            int aValue  //!< value
            );


        void pollMotors();

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


    //************************************************************************
    // variables
    //************************************************************************
    private:
        Ui::TmcmBb4*                    mMainUi;            //!< main UI

        CanOpen*                        mCanOpen;           //!< CANopen instance
        bool                            mStartAutoConnect;  //!< if CAN autoconnect is active

        std::vector<Tmcm1637CanOpen>    mTmcm1637Vec;       //!< motion controllers
        std::vector<bool>               mTmcmIdentityVec;   //!< identity vector
        std::vector<bool>               mTmcmStateVec;      //!< state vector
        std::vector<bool>               mTmcmOperEnVec;     //!< operation enabled vector

        QTimer*                         mMotorsPollingTimer;//!< timer for polling the motors
};

#endif // TmcmBb4_h
