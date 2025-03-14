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
CanOpen.hpp

This file contains the definitions for CANopen.
*/

#ifndef CanOpen_hpp
#define CanOpen_hpp

#include <QCanBusDevice>
#include <QCanBusDeviceInfo>
#include <QCanBusFrame>
#include <QObject>
#include <QPair>
#include <QString>
#include <QTimer>
#include <QVariant>

#include <cstdint>
#include <map>
#include <memory>
#include <string>


//************************************************************************
// Class for handling CANopen
//************************************************************************
class CanOpen : public QObject
{
    Q_OBJECT

    //************************************************************************
    // constants and types
    //************************************************************************
    public:
        static const uint16_t CAN_ID_BASE_TX = 0x600;   //!< for COB-ID @ Tx
        static const uint16_t CAN_ID_BASE_RX = 0x580;   //!< for COB-ID @ Rx

        static const int BIT_RATE;

        typedef enum : uint8_t
        {
            SDO_CMD_READ                = 0x40,
            SDO_CMD_WRITE_BYTE          = 0x2F,
            SDO_CMD_WRITE_WORD          = 0x2B,
            SDO_CMD_WRITE_DOUBLE_WORD   = 0x23
        }SdoCmd;

        typedef enum : uint8_t
        {
            SDO_ANSWER_WRITE_OK         = 0x60,
            SDO_ANSWER_READ_BYTE        = 0x4F,
            SDO_ANSWER_READ_WORD        = 0x4B,
            SDO_ANSWER_READ_DOUBLE_WORD = 0x43
        }SdoAnswer;

        typedef enum : uint8_t
        {
            CAN_BUS_STATUS_UNKNOWN,

            CAN_BUS_STATUS_OK,
            CAN_BUS_STATUS_WARNING,
            CAN_BUS_STATUS_ERROR,
            CAN_BUS_STATUS_OFF
        }CanBusStatus;

        static const std::map<CanBusStatus, std::string> CAN_BUS_STATUS_NAMES;

    private:
        typedef QPair<QCanBusDevice::ConfigurationKey, QVariant> ConfigurationItem;

        typedef struct
        {
            QString                     pluginName;
            QString                     deviceName;
            QList<ConfigurationItem>    configList;
        }CanSettings;

        static const int FRAME_RX_TIMEOUT_MS = 50;  // ms at 1 Mbps

        static const unsigned long DELAY_BEFORE_WR_US = 120;  // us


    //************************************************************************
    // functions
    //************************************************************************
    public:
        CanOpen();

        ~CanOpen();

        static CanOpen* getInstance();

        void connectProcessRxFrames
            (
            bool aStatus    //!< true if connecting the Rx processing
            );

        bool createAndSendFrame
            (
            const uint16_t  aFrameId,   //!< frame ID
            const uint16_t  aObject,    //!< CANopen object
            const uint8_t   aSubindex,  //!< subindex
            const uint8_t   aByte       //!< payload data
            ) const;

        bool createAndSendFrame
            (
            const uint16_t  aFrameId,   //!< frame ID
            const uint16_t  aObject,    //!< CANopen object
            const uint8_t   aSubindex,  //!< subindex
            const int8_t    aSignedByte //!< payload data
            ) const;

        bool createAndSendFrame
            (
            const uint16_t  aFrameId,   //!< frame ID
            const uint16_t  aObject,    //!< CANopen object
            const uint8_t   aSubindex,  //!< subindex
            const uint16_t  aWord       //!< payload data
            ) const;

        bool createAndSendFrame
            (
            const uint16_t  aFrameId,   //!< frame ID
            const uint16_t  aObject,    //!< CANopen object
            const uint8_t   aSubindex,  //!< subindex
            const int16_t   aSignedWord //!< payload data
            ) const;

        bool createAndSendFrame
            (
            const uint16_t  aFrameId,   //!< frame ID
            const uint16_t  aObject,    //!< CANopen object
            const uint8_t   aSubindex,  //!< subindex
            const uint32_t  aDoubleWord //!< payload data
            ) const;

        bool createAndSendFrame
            (
            const uint16_t  aFrameId,           //!< frame ID
            const uint16_t  aObject,            //!< CANopen object
            const uint8_t   aSubindex,          //!< subindex
            const int32_t   aSignedDoubleWord   //!< payload data
            ) const;

        void disconnect();

        QCanBusDevice* getCanDevice();

        bool initCanBus
            (
            const bool aLoopbackEnabled = false //!< loopback status
            );

        bool isConnected() const;

        bool readFrame
            (
            const uint16_t  aFrameId,   //!< frame ID
            const uint16_t  aObject,    //!< CANopen object
            const uint8_t   aSubindex,  //!< subindex
            uint8_t&        aLength,    //!< length of read data
            uint32_t&       aDoubleWord //!< read payload data
            );

        void reset() const;

        bool sendFrame
            (
            const QCanBusFrame aFrame   //!< CAN frame
            ) const;        

        bool testFrame
            (
            const uint16_t  aFrameId,   //!< frame ID
            const uint16_t  aObject,    //!< CANopen object
            const uint8_t   aSubindex,  //!< subindex
            const uint32_t  aDoubleWord,//!< payload data
            const bool      aLoopback   //!< loopback status
            );

    signals:
        void connectionStatusChanged
            (
            bool aStatus                //!< true if conected
            );

        void connectionStrChanged
            (
            QString aString             //!< connection string
            );

        void receivedFrame
            (
            QString aString             //!< formatted frame
            );

        void statusStrChanged
            (
            QString aString             //!< status string
            );

    private:
        void updateStatus();

    private slots:
        void processErrors
            (
            QCanBusDevice::CanBusError          aError  //!< CAN error
            );

        void processRxFrames();

        void processStateChanged
            (
            QCanBusDevice::CanBusDeviceState    aState  //!< CAN state
            );


    //************************************************************************
    // variables
    //************************************************************************
    private:
        static CanOpen*                 sInstance;          //!< singleton

        std::unique_ptr<QCanBusDevice>  mCanDevice;         //!< CAN device
        bool                            mIsConnected;       //!< if connected to the CAN bus
        bool                            mLoopbackEnabled;   //!< if loopback is enabled
        bool                            mProcessRxFrames;   //!< if processing Rx frames
        CanBusStatus                    mStatus;            //!< CAN bus status
        CanSettings                     mSettings;          //!< CAN settings
        QTimer*                         mStatusTimer;       //!< timer for CAN status

        QString                         mConnectionStr;     //!< connection string
        QString                         mStatusStr;         //!< status string
};

#endif // CanOpen_hpp
