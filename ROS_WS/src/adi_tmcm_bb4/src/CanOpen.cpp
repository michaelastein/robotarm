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
CanOpen.cpp

This file contains the sources for CANopen.
*/

#include <adi_tmcm_bb4/CanOpen.hpp>

#include <QCanBus>
#include <QThread>

#include <vector>


const int CanOpen::BIT_RATE = 1000000;

CanOpen* CanOpen::sInstance = nullptr;

const std::map<CanOpen::CanBusStatus, std::string> CanOpen::CAN_BUS_STATUS_NAMES =
{
    { CanOpen::CAN_BUS_STATUS_UNKNOWN, "Unknown" },
    { CanOpen::CAN_BUS_STATUS_OK,      "OK"      },
    { CanOpen::CAN_BUS_STATUS_WARNING, "Warning" },
    { CanOpen::CAN_BUS_STATUS_ERROR,   "Error"   },
    { CanOpen::CAN_BUS_STATUS_OFF,     "Off"     }
};

//!************************************************************************
//! Constructor
//!************************************************************************
CanOpen::CanOpen()
    : mIsConnected( false )
    , mLoopbackEnabled( false )
    , mProcessRxFrames( false )
    , mStatus( CAN_BUS_STATUS_UNKNOWN )
    , mStatusTimer( new QTimer() )
{
    connect( mStatusTimer, &QTimer::timeout, this, &CanOpen::updateStatus );
}


//!************************************************************************
//! Destructor
//!************************************************************************
CanOpen::~CanOpen()
{
    if( mCanDevice )
    {
        mCanDevice->disconnectDevice();
    }
}


//!************************************************************************
//! Singleton
//!
//! @returns the instance of the object
//!************************************************************************
CanOpen* CanOpen::getInstance()
{
    if( !sInstance )
    {
        sInstance = new CanOpen;
    }

    return sInstance;
}


//!************************************************************************
//! Connect or disconnect the Rx frames processing
//!
//! @returns nothing
//!************************************************************************
void CanOpen::connectProcessRxFrames
    (
    bool aStatus    //!< true if connecting the Rx processing
    )
{
    if( mCanDevice )
    {
        mProcessRxFrames = aStatus;

        if( mProcessRxFrames )
        {            
            connect( mCanDevice.get(), &QCanBusDevice::framesReceived, this, &CanOpen::processRxFrames );
        }
        else
        {
            QObject::disconnect( mCanDevice.get(), &QCanBusDevice::framesReceived, nullptr, nullptr );
        }
    }
}


//!************************************************************************
//! Create a CAN frame and send it
//! The payload is using one byte.
//!
//! @returns true if the CAN answer is Write_OK
//!************************************************************************
bool CanOpen::createAndSendFrame
    (
    const uint16_t  aFrameId,   //!< frame ID
    const uint16_t  aObject,    //!< CANopen object
    const uint8_t   aSubindex,  //!< subindex
    const uint8_t   aByte       //!< payload data
    ) const
{
    QByteArray payload;
    payload.resize( 5 );
    payload[0] = CanOpen::SDO_CMD_WRITE_BYTE;
    payload[1] = aObject & 0xFF;
    payload[2] = aObject >> 8;
    payload[3] = aSubindex;
    payload[4] = aByte;
    QCanBusFrame txFrame = QCanBusFrame( aFrameId, payload );
    QThread::usleep( DELAY_BEFORE_WR_US );
    bool status = mCanDevice->writeFrame( txFrame );

    QCanBusFrame rxFrame;
    QByteArray rxLoadByteArray;
    size_t rxLoadLen = 0;

    if( status )
    {
        status = mCanDevice->waitForFramesReceived( FRAME_RX_TIMEOUT_MS );
    }

    if( status )
    {
        rxFrame = mCanDevice->readFrame();
        status = ( QCanBusFrame::DataFrame == rxFrame.frameType() );
    }
    else
    {
        QThread::usleep( DELAY_BEFORE_WR_US );
        rxFrame = mCanDevice->readFrame();
    }

    if( status )
    {
        uint16_t rxCanID = static_cast<uint16_t>( rxFrame.frameId() );
        status = ( aFrameId - CAN_ID_BASE_TX == rxCanID - CAN_ID_BASE_RX );
    }

    if( status )
    {
        rxLoadByteArray = rxFrame.payload();
        rxLoadLen = rxLoadByteArray.size();
        status = ( rxLoadLen >= 4 );
    }

    if( status )
    {
        std::vector<uint8_t> rxLoadVec( rxLoadLen );

        for( size_t i = 0; i < rxLoadLen; i++ )
        {
            rxLoadVec.at( i ) = static_cast<uint8_t>( rxLoadByteArray.data()[i] );
        }

        status = ( SDO_ANSWER_WRITE_OK == rxLoadVec.at( 0 ) )
              && ( ( aObject & 0xFF ) == rxLoadVec.at( 1 ) )
              && ( ( aObject >> 8 ) == rxLoadVec.at( 2 ) )
              && ( aSubindex == rxLoadVec.at( 3 ) );
    }

    return status;
}


//!************************************************************************
//! Create a CAN frame and send it
//! The payload is using one signed byte.
//!
//! @returns true if the CAN answer is Write_OK
//!************************************************************************
bool CanOpen::createAndSendFrame
    (
    const uint16_t  aFrameId,   //!< frame ID
    const uint16_t  aObject,    //!< CANopen object
    const uint8_t   aSubindex,  //!< subindex
    const int8_t    aSignedByte //!< payload data
    ) const
{
    QByteArray payload;
    payload.resize( 5 );
    payload[0] = CanOpen::SDO_CMD_WRITE_BYTE;
    payload[1] = aObject & 0xFF;
    payload[2] = aObject >> 8;
    payload[3] = aSubindex;
    payload[4] = aSignedByte;
    QCanBusFrame txFrame = QCanBusFrame( aFrameId, payload );
    QThread::usleep( DELAY_BEFORE_WR_US );
    bool status = mCanDevice->writeFrame( txFrame );

    QCanBusFrame rxFrame;
    QByteArray rxLoadByteArray;
    size_t rxLoadLen = 0;

    if( status )
    {
        status = mCanDevice->waitForFramesReceived( FRAME_RX_TIMEOUT_MS );
    }

    if( status )
    {
        rxFrame = mCanDevice->readFrame();
        status = ( QCanBusFrame::DataFrame == rxFrame.frameType() );
    }
    else
    {
        QThread::usleep( DELAY_BEFORE_WR_US );
        rxFrame = mCanDevice->readFrame();
    }

    if( status )
    {
        uint16_t rxCanID = static_cast<uint16_t>( rxFrame.frameId() );
        status = ( aFrameId - CAN_ID_BASE_TX == rxCanID - CAN_ID_BASE_RX );
    }

    if( status )
    {
        rxLoadByteArray = rxFrame.payload();
        rxLoadLen = rxLoadByteArray.size();
        status = ( rxLoadLen >= 4 );
    }

    if( status )
    {
        std::vector<uint8_t> rxLoadVec( rxLoadLen );

        for( size_t i = 0; i < rxLoadLen; i++ )
        {
            rxLoadVec.at( i ) = static_cast<uint8_t>( rxLoadByteArray.data()[i] );
        }

        status = ( SDO_ANSWER_WRITE_OK == rxLoadVec.at( 0 ) )
              && ( ( aObject & 0xFF ) == rxLoadVec.at( 1 ) )
              && ( ( aObject >> 8 ) == rxLoadVec.at( 2 ) )
              && ( aSubindex == rxLoadVec.at( 3 ) );
    }

    return status;
}


//!************************************************************************
//! Create a CAN frame and send it
//! The payload is using one word.
//!
//! @returns true if the CAN answer is Write_OK
//!************************************************************************
bool CanOpen::createAndSendFrame
    (
    const uint16_t  aFrameId,   //!< frame ID
    const uint16_t  aObject,    //!< CANopen object
    const uint8_t   aSubindex,  //!< subindex
    const uint16_t  aWord       //!< payload data
    ) const
{
    QByteArray payload;
    payload.resize( 6 );
    payload[0] = CanOpen::SDO_CMD_WRITE_WORD;
    payload[1] = aObject & 0xFF;
    payload[2] = aObject >> 8;
    payload[3] = aSubindex;
    payload[4] = aWord & 0xFF;
    payload[5] = aWord >> 8;
    QCanBusFrame txFrame = QCanBusFrame( aFrameId, payload );
    QThread::usleep( DELAY_BEFORE_WR_US );
    bool status = mCanDevice->writeFrame( txFrame );

    QCanBusFrame rxFrame;
    QByteArray rxLoadByteArray;
    size_t rxLoadLen = 0;

    if( status )
    {
        status = mCanDevice->waitForFramesReceived( FRAME_RX_TIMEOUT_MS );
    }

    if( status )
    {
        rxFrame = mCanDevice->readFrame();
        status = ( QCanBusFrame::DataFrame == rxFrame.frameType() );
    }
    else
    {
        QThread::usleep( DELAY_BEFORE_WR_US );
        rxFrame = mCanDevice->readFrame();
    }

    if( status )
    {
        uint16_t rxCanID = static_cast<uint16_t>( rxFrame.frameId() );
        status = ( aFrameId - CAN_ID_BASE_TX == rxCanID - CAN_ID_BASE_RX );
    }

    if( status )
    {
        rxLoadByteArray = rxFrame.payload();
        rxLoadLen = rxLoadByteArray.size();
        status = ( rxLoadLen >= 4 );
    }

    if( status )
    {
        std::vector<uint8_t> rxLoadVec( rxLoadLen );

        for( size_t i = 0; i < rxLoadLen; i++ )
        {
            rxLoadVec.at( i ) = static_cast<uint8_t>( rxLoadByteArray.data()[i] );
        }

        status = ( SDO_ANSWER_WRITE_OK == rxLoadVec.at( 0 ) )
              && ( ( aObject & 0xFF ) == rxLoadVec.at( 1 ) )
              && ( ( aObject >> 8 ) == rxLoadVec.at( 2 ) )
              && ( aSubindex == rxLoadVec.at( 3 ) );
    }

    return status;
}


//!************************************************************************
//! Create a CAN frame and send it
//! The payload is using one signed word.
//!
//! @returns true if the CAN answer is Write_OK
//!************************************************************************
bool CanOpen::createAndSendFrame
    (
    const uint16_t  aFrameId,   //!< frame ID
    const uint16_t  aObject,    //!< CANopen object
    const uint8_t   aSubindex,  //!< subindex
    const int16_t   aSignedWord //!< payload data
    ) const
{
    QByteArray payload;
    payload.resize( 6 );
    payload[0] = CanOpen::SDO_CMD_WRITE_WORD;
    payload[1] = aObject & 0xFF;
    payload[2] = aObject >> 8;
    payload[3] = aSubindex;
    payload[4] = aSignedWord & 0xFF;
    payload[5] = aSignedWord >> 8;
    QCanBusFrame txFrame = QCanBusFrame( aFrameId, payload );
    QThread::usleep( DELAY_BEFORE_WR_US );
    bool status = mCanDevice->writeFrame( txFrame );

    QCanBusFrame rxFrame;
    QByteArray rxLoadByteArray;
    size_t rxLoadLen = 0;

    if( status )
    {
        status = mCanDevice->waitForFramesReceived( FRAME_RX_TIMEOUT_MS );
    }

    if( status )
    {
        rxFrame = mCanDevice->readFrame();
        status = ( QCanBusFrame::DataFrame == rxFrame.frameType() );
    }
    else
    {
        QThread::usleep( DELAY_BEFORE_WR_US );
        rxFrame = mCanDevice->readFrame();
    }        

    if( status )
    {
        uint16_t rxCanID = static_cast<uint16_t>( rxFrame.frameId() );
        status = ( aFrameId - CAN_ID_BASE_TX == rxCanID - CAN_ID_BASE_RX );
    }

    if( status )
    {
        rxLoadByteArray = rxFrame.payload();
        rxLoadLen = rxLoadByteArray.size();
        status = ( rxLoadLen >= 4 );
    }

    if( status )
    {
        std::vector<uint8_t> rxLoadVec( rxLoadLen );

        for( size_t i = 0; i < rxLoadLen; i++ )
        {
            rxLoadVec.at( i ) = static_cast<uint8_t>( rxLoadByteArray.data()[i] );
        }

        status = ( SDO_ANSWER_WRITE_OK == rxLoadVec.at( 0 ) )
              && ( ( aObject & 0xFF ) == rxLoadVec.at( 1 ) )
              && ( ( aObject >> 8 ) == rxLoadVec.at( 2 ) )
              && ( aSubindex == rxLoadVec.at( 3 ) );
    }

    return status;
}


//!************************************************************************
//! Create a CAN frame and send it
//! The payload is using one double word.
//!
//! @returns true if the CAN answer is Write_OK
//!************************************************************************
bool CanOpen::createAndSendFrame
    (
    const uint16_t  aFrameId,   //!< frame ID
    const uint16_t  aObject,    //!< CANopen object
    const uint8_t   aSubindex,  //!< subindex
    const uint32_t  aDoubleWord //!< payload data
    ) const
{    
    QByteArray payload;
    payload.resize( 8 );
    payload[0] = CanOpen::SDO_CMD_WRITE_DOUBLE_WORD;
    payload[1] = aObject & 0xFF;
    payload[2] = aObject >> 8;
    payload[3] = aSubindex;
    payload[4] = ( aDoubleWord & 0x000000FF );
    payload[5] = ( aDoubleWord & 0x0000FF00 ) >> 8;
    payload[6] = ( aDoubleWord & 0x00FF0000 ) >> 16;
    payload[7] = ( aDoubleWord & 0xFF000000 ) >> 24;
    QCanBusFrame txFrame = QCanBusFrame( aFrameId, payload );
    QThread::usleep( DELAY_BEFORE_WR_US );
    bool status = mCanDevice->writeFrame( txFrame );

    QCanBusFrame rxFrame;
    QByteArray rxLoadByteArray;
    size_t rxLoadLen = 0;

    if( status )
    {
        status = mCanDevice->waitForFramesReceived( FRAME_RX_TIMEOUT_MS );
    }

    if( status )
    {
        rxFrame = mCanDevice->readFrame();
        status = ( QCanBusFrame::DataFrame == rxFrame.frameType() );
    }
    else
    {
        QThread::usleep( DELAY_BEFORE_WR_US );
        rxFrame = mCanDevice->readFrame();
    }

    if( status )
    {
        uint16_t rxCanID = static_cast<uint16_t>( rxFrame.frameId() );
        status = ( aFrameId - CAN_ID_BASE_TX == rxCanID - CAN_ID_BASE_RX );
    }

    if( status )
    {
        rxLoadByteArray = rxFrame.payload();
        rxLoadLen = rxLoadByteArray.size();
        status = ( rxLoadLen >= 4 );
    }

    if( status )
    {
        std::vector<uint8_t> rxLoadVec( rxLoadLen );

        for( size_t i = 0; i < rxLoadLen; i++ )
        {
            rxLoadVec.at( i ) = static_cast<uint8_t>( rxLoadByteArray.data()[i] );
        }

        status = ( SDO_ANSWER_WRITE_OK == rxLoadVec.at( 0 ) )
              && ( ( aObject & 0xFF ) == rxLoadVec.at( 1 ) )
              && ( ( aObject >> 8 ) == rxLoadVec.at( 2 ) )
              && ( aSubindex == rxLoadVec.at( 3 ) );
    }

    return status;
}


//!************************************************************************
//! Create a CAN frame and send it
//! The payload is using one signed double word.
//!
//! @returns true if the CAN answer is Write_OK
//!************************************************************************
bool CanOpen::createAndSendFrame
    (
    const uint16_t  aFrameId,           //!< frame ID
    const uint16_t  aObject,            //!< CANopen object
    const uint8_t   aSubindex,          //!< subindex
    const int32_t   aSignedDoubleWord   //!< payload data
    ) const
{
    QByteArray payload;
    payload.resize( 8 );
    payload[0] = CanOpen::SDO_CMD_WRITE_DOUBLE_WORD;
    payload[1] = aObject & 0xFF;
    payload[2] = aObject >> 8;
    payload[3] = aSubindex;
    payload[4] = ( aSignedDoubleWord & 0x000000FF );
    payload[5] = ( aSignedDoubleWord & 0x0000FF00 ) >> 8;
    payload[6] = ( aSignedDoubleWord & 0x00FF0000 ) >> 16;
    payload[7] = ( aSignedDoubleWord & 0xFF000000 ) >> 24;
    QCanBusFrame txFrame = QCanBusFrame( aFrameId, payload );
    QThread::usleep( DELAY_BEFORE_WR_US );
    bool status = mCanDevice->writeFrame( txFrame );

    QCanBusFrame rxFrame;
    QByteArray rxLoadByteArray;
    size_t rxLoadLen = 0;

    if( status )
    {
        status = mCanDevice->waitForFramesReceived( FRAME_RX_TIMEOUT_MS );
    }

    if( status )
    {
        rxFrame = mCanDevice->readFrame();
        status = ( QCanBusFrame::DataFrame == rxFrame.frameType() );
    }
    else
    {
        QThread::usleep( DELAY_BEFORE_WR_US );
        rxFrame = mCanDevice->readFrame();
    }   

    if( status )
    {
        uint16_t rxCanID = static_cast<uint16_t>( rxFrame.frameId() );
        status = ( aFrameId - CAN_ID_BASE_TX == rxCanID - CAN_ID_BASE_RX );
    }

    if( status )
    {
        rxLoadByteArray = rxFrame.payload();
        rxLoadLen = rxLoadByteArray.size();
        status = ( rxLoadLen >= 4 );
    }

    if( status )
    {
        std::vector<uint8_t> rxLoadVec( rxLoadLen );

        for( size_t i = 0; i < rxLoadLen; i++ )
        {
            rxLoadVec.at( i ) = static_cast<uint8_t>( rxLoadByteArray.data()[i] );
        }

        status = ( SDO_ANSWER_WRITE_OK == rxLoadVec.at( 0 ) )
              && ( ( aObject & 0xFF ) == rxLoadVec.at( 1 ) )
              && ( ( aObject >> 8 ) == rxLoadVec.at( 2 ) )
              && ( aSubindex == rxLoadVec.at( 3 ) );
    }

    return status;
}


//!************************************************************************
//! Disconnect the device from CAN bus
//!
//! @returns nothing
//!************************************************************************
void CanOpen::disconnect()
{
    if( mCanDevice )
    {
        mCanDevice->disconnectDevice();
    }

    mStatusTimer->stop();
    mIsConnected = false;
    mConnectionStr = "CAN bus disconnected.";
    mStatusStr.clear();
    emit connectionStatusChanged( mIsConnected );
    emit connectionStrChanged( mConnectionStr );
    emit statusStrChanged( mStatusStr );
}


//!************************************************************************
//! Get the CAN device object
//!
//! @returns a pointer to a QCanBusDevice object
//!************************************************************************
QCanBusDevice* CanOpen::getCanDevice()
{
    return mCanDevice.get();
}


//!************************************************************************
//! Initialize the CAN bus
//!
//! @returns: true if initialization can be done
//!************************************************************************
bool CanOpen::initCanBus
    (
    const bool aLoopbackEnabled //!< loopback status
    )
{
    mLoopbackEnabled = aLoopbackEnabled;
    bool status = false;

    if( mCanDevice && mIsConnected )
    {
        mCanDevice->disconnectDevice();
    }

#if defined( __linux__ ) && defined( __x86_64__ )
    mSettings.pluginName = "socketcan";
    mSettings.deviceName = "can0";
#elif defined( __linux__ ) && defined( __aarch64__ )
    mSettings.pluginName = "peakcan";
    mSettings.deviceName = "usb0";
#elif defined( _WIN32 ) || defined( WIN32 )
    mSettings.pluginName = "peakcan";
    mSettings.deviceName = "usb0";
#endif

    mSettings.configList.clear();
    mSettings.configList.append( ConfigurationItem( QCanBusDevice::CanFdKey, false ) );
    mSettings.configList.append( ConfigurationItem( QCanBusDevice::LoopbackKey, mLoopbackEnabled ) );
    mSettings.configList.append( ConfigurationItem( QCanBusDevice::ReceiveOwnKey, mLoopbackEnabled ) );
    mSettings.configList.append( ConfigurationItem( QCanBusDevice::BitRateKey, BIT_RATE ) );

    QString errorString;
    QString trimmedErrorString;
    mCanDevice.reset( QCanBus::instance()->createDevice( mSettings.pluginName, mSettings.deviceName, &errorString ) );

    if( mCanDevice )
    {
        for( const ConfigurationItem &item : mSettings.configList )
        {
            mCanDevice->setConfigurationParameter( item.first, item.second );
        }

        connect( mCanDevice.get(), &QCanBusDevice::stateChanged, this, &CanOpen::processStateChanged );
        connect( mCanDevice.get(), &QCanBusDevice::errorOccurred, this, &CanOpen::processErrors );

        if( mProcessRxFrames )
        {
            connect( mCanDevice.get(), &QCanBusDevice::framesReceived, this, &CanOpen::processRxFrames );
        }
        else
        {
            QObject::disconnect( mCanDevice.get(), &QCanBusDevice::framesReceived, nullptr, nullptr );
        }

        if( mCanDevice->connectDevice() )
        {
            const QVariant bitRate = mCanDevice->configurationParameter( QCanBusDevice::BitRateKey );
            QString brKbps = QString::number( bitRate.toInt() / 1000 );

            if( bitRate.isValid() )
            {
                mConnectionStr = "CAN plugin: " + mSettings.pluginName
                        + ", connected to " + mSettings.deviceName
                        + " at " + brKbps + " kbps";
            }
            else
            {
                mConnectionStr = brKbps + " kbps not supported";
            }

            updateStatus();
            mStatusTimer->start( 500 );

            mIsConnected = true;
            status = true;
        }
        else
        {
            errorString = mCanDevice->errorString();
            trimmedErrorString = errorString.left( errorString.indexOf( '\0' ) );
            mConnectionStr = "Error connecting to CAN bus: " + trimmedErrorString;
            mCanDevice.reset();
        }
    }
    else
    {
        trimmedErrorString = errorString.left( errorString.indexOf( '\0' ) );
        mConnectionStr = "Error initializing CAN bus: " + trimmedErrorString;
    }

    emit connectionStatusChanged( mIsConnected );
    emit connectionStrChanged( mConnectionStr );

    return status;
}


//!************************************************************************
//! Check if connected to the CAN bus
//!
//! @returns true if connected
//!************************************************************************
bool CanOpen::isConnected() const
{
    return mIsConnected;
}


//!************************************************************************
//! Process the CAN errors
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void CanOpen::processErrors
    (
    QCanBusDevice::CanBusError  aError  //!< CAN error
    )
{
    QString errorString = mCanDevice->errorString();
    errorString = errorString.left( errorString.indexOf( '\0' ) );

    switch( aError )
    {
        case QCanBusDevice::ReadError:
        case QCanBusDevice::WriteError:
        case QCanBusDevice::ConnectionError:
        case QCanBusDevice::ConfigurationError:
        case QCanBusDevice::UnknownError:
            mConnectionStr = errorString;
            emit connectionStrChanged( mConnectionStr );
            break;

        default:
            break;
    }
}


//!************************************************************************
//! Process the received CAN frames
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void CanOpen::processRxFrames()
{
    if( mCanDevice )
    {
        if( !mLoopbackEnabled )
        {
            while( mCanDevice->framesAvailable() )
            {
                uint16_t rxFrameId = 0;
                QByteArray rxLoadByteArray;
                size_t rxLoadLen = 0;
                QCanBusFrame rxFrame = mCanDevice->readFrame();

                bool status = ( QCanBusFrame::DataFrame == rxFrame.frameType() );

                if( status )
                {
                    rxFrameId = static_cast<uint16_t>( rxFrame.frameId() );
                    rxLoadByteArray = rxFrame.payload();
                    rxLoadLen = rxLoadByteArray.size();
                    status = ( rxLoadLen >= 4 );
                }

                if( status )
                {
                    QByteArray txLoadByteArray = rxLoadByteArray;

                    for( size_t i = 4; i < rxLoadLen; i++ )
                    {
                        uint8_t crtByte = static_cast<uint8_t>( txLoadByteArray.at( i ) );
                        txLoadByteArray.data()[i] = static_cast<char>( ~crtByte );
                    }

                    QCanBusFrame txFrame = QCanBusFrame( rxFrameId, txLoadByteArray );
                    QThread::usleep( DELAY_BEFORE_WR_US );
                    status = mCanDevice->writeFrame( txFrame );
                }
            }
        }
    }
}


//!************************************************************************
//! Process the CAN state
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void CanOpen::processStateChanged
   (
   QCanBusDevice::CanBusDeviceState    aState  //!< CAN state
   )
{
    switch( aState )
    {
        case QCanBusDevice::UnconnectedState:
            mIsConnected = false;
            mConnectionStr = "CAN bus disconnected.";
            mStatusStr.clear();
            emit connectionStatusChanged( mIsConnected );
            emit connectionStrChanged( mConnectionStr );
            emit statusStrChanged( mStatusStr );
            break;

        default:
            break;
    }
}


//!************************************************************************
//! Read a CAN frame
//!
//! @returns true if the read can be done
//!************************************************************************
bool CanOpen::readFrame
    (
    const uint16_t  aFrameId,   //!< frame ID
    const uint16_t  aObject,    //!< CANopen object
    const uint8_t   aSubindex,  //!< subindex
    uint8_t&        aLength,    //!< length of read data
    uint32_t&       aDoubleWord //!< read payload data
    )
{
    bool status = false;
    aLength = 0;
    aDoubleWord = 0;

    if( mCanDevice )
    {
        QByteArray payload;
        payload.resize( 4 );
        payload[0] = CanOpen::SDO_CMD_READ;
        payload[1] = aObject & 0xFF;
        payload[2] = aObject >> 8;
        payload[3] = aSubindex;
        QCanBusFrame txFrame = QCanBusFrame( aFrameId, payload );
        QThread::usleep( DELAY_BEFORE_WR_US );
        status = mCanDevice->writeFrame( txFrame );

        QCanBusFrame rxFrame;
        QByteArray rxLoadByteArray;
        size_t rxLoadLen = 0;
        std::vector<uint8_t> rxLoadVec;

        if( status )
        {
            status = mCanDevice->waitForFramesReceived( FRAME_RX_TIMEOUT_MS );
        }

        if( status )
        {
            rxFrame = mCanDevice->readFrame();
            status = QCanBusFrame::DataFrame == rxFrame.frameType();
        }
        else
        {
            QThread::usleep( DELAY_BEFORE_WR_US );
            rxFrame = mCanDevice->readFrame();
        }

        if( status )
        {
            uint16_t rxCanID = static_cast<uint16_t>( rxFrame.frameId() );
            status = ( aFrameId - CAN_ID_BASE_TX == rxCanID - CAN_ID_BASE_RX );
        }

        if( status )
        {
            rxLoadByteArray = rxFrame.payload();
            rxLoadLen = rxLoadByteArray.size();
            status = ( rxLoadLen >= 5 );
        }

        if( status )
        {
            rxLoadVec.resize( rxLoadLen );

            for( size_t i = 0; i < rxLoadLen; i++ )
            {
                rxLoadVec.at( i ) = static_cast<uint8_t>( rxLoadByteArray.data()[i] );
            }

            status = ( ( aObject & 0xFF ) == rxLoadVec.at( 1 ) )
                  && ( ( aObject >> 8 ) == rxLoadVec.at( 2 ) )
                  && ( aSubindex == rxLoadVec.at( 3 ) );
        }

        if( status )
        {
            switch( rxLoadVec.at( 0 ) )
            {
                case SDO_ANSWER_READ_BYTE:
                    aLength = 1;
                    aDoubleWord = rxLoadVec.at( 4 );
                    break;

                case SDO_ANSWER_READ_WORD:
                    aLength = 2;
                    aDoubleWord = rxLoadVec.at( 4 );
                    aDoubleWord |= rxLoadVec.at( 5 ) << 8;
                    break;

                case SDO_ANSWER_READ_DOUBLE_WORD:
                    aLength = 4;
                    aDoubleWord = rxLoadVec.at( 4 );
                    aDoubleWord |= rxLoadVec.at( 5 ) << 8;
                    aDoubleWord |= rxLoadVec.at( 6 ) << 16;
                    aDoubleWord |= rxLoadVec.at( 7 ) << 24;
                    break;

                default:
                    status = false;
                    break;
            }
        }
    }

    return status;
}


//!************************************************************************
//! Reset the CAN controller
//!
//! @returns nothing
//!************************************************************************
void CanOpen::reset() const
{
    if( mCanDevice )
    {
        mCanDevice->resetController();
    }
}


//!************************************************************************
//! Send a CAN frame
//!
//! @returns: true if the frame can be sent
//!************************************************************************
bool CanOpen::sendFrame
    (
    const QCanBusFrame aFrame   //!< CAN frame
    ) const
{
    bool status = false;

    if( mCanDevice && mIsConnected )
    {
        QThread::usleep( DELAY_BEFORE_WR_US );
        status = mCanDevice->writeFrame( aFrame );
    }

    return status;
}


//!************************************************************************
//! Test a CAN frame between two devices
//!
//! @returns true if the test passed
//!************************************************************************
bool CanOpen::testFrame
    (
    const uint16_t  aFrameId,   //!< frame ID
    const uint16_t  aObject,    //!< CANopen object
    const uint8_t   aSubindex,  //!< subindex
    const uint32_t  aDoubleWord,//!< payload data
    const bool      aLoopback   //!< loopback status
    )
{
    QByteArray payload;
    payload.resize( 8 );
    payload[0] = CanOpen::SDO_CMD_WRITE_DOUBLE_WORD;
    payload[1] = aObject & 0xFF;
    payload[2] = aObject >> 8;
    payload[3] = aSubindex;
    payload[4] = ( aDoubleWord & 0x000000FF );
    payload[5] = ( aDoubleWord & 0x0000FF00 ) >> 8;
    payload[6] = ( aDoubleWord & 0x00FF0000 ) >> 16;
    payload[7] = ( aDoubleWord & 0xFF000000 ) >> 24;
    QCanBusFrame txFrame = QCanBusFrame( aFrameId, payload );
    QThread::usleep( DELAY_BEFORE_WR_US );
    bool status = mCanDevice->writeFrame( txFrame );

    QCanBusFrame rxFrame;
    QByteArray rxLoadByteArray;
    size_t rxLoadLen = 0;

    if( status )
    {
        status = mCanDevice->waitForFramesReceived( FRAME_RX_TIMEOUT_MS );
    }

    if( status )
    {
        rxFrame = mCanDevice->readFrame();
        status = ( QCanBusFrame::DataFrame == rxFrame.frameType() );
    }
    else
    {
        QThread::usleep( DELAY_BEFORE_WR_US );
        rxFrame = mCanDevice->readFrame();
    }      

    if( status )
    {
        uint16_t rxCanID = static_cast<uint16_t>( rxFrame.frameId() );
        status = ( aFrameId == rxCanID );
    }

    if( status )
    {
        rxLoadByteArray = rxFrame.payload();
        rxLoadLen = rxLoadByteArray.size();
        status = ( rxLoadLen >= 4 );
    }

    if( status )
    {
        std::vector<uint8_t> rxLoadVec( rxLoadLen );

        for( size_t i = 0; i < rxLoadLen; i++ )
        {
            rxLoadVec.at( i ) = static_cast<uint8_t>( rxLoadByteArray.data()[i] );
        }

        uint8_t expectedB4 = ( aDoubleWord & 0x000000FF );
        uint8_t expectedB5 = ( aDoubleWord & 0x0000FF00 ) >> 8;
        uint8_t expectedB6 = ( aDoubleWord & 0x00FF0000 ) >> 16;
        uint8_t expectedB7 = ( aDoubleWord & 0xFF000000 ) >> 24;

        if( !aLoopback )
        {
            expectedB4 = ~expectedB4;
            expectedB5 = ~expectedB5;
            expectedB6 = ~expectedB6;
            expectedB7 = ~expectedB7;
        }

        status = ( CanOpen::SDO_CMD_WRITE_DOUBLE_WORD == rxLoadVec.at( 0 ) )
              && ( ( aObject & 0xFF ) == rxLoadVec.at( 1 ) )
              && ( ( aObject >> 8 ) == rxLoadVec.at( 2 ) )
              && ( aSubindex == rxLoadVec.at( 3 ) )
              && ( expectedB4 == rxLoadVec.at( 4 ) )
              && ( expectedB5 == rxLoadVec.at( 5 ) )
              && ( expectedB6 == rxLoadVec.at( 6 ) )
              && ( expectedB7 == rxLoadVec.at( 7 ) );
    }

    return status;
}


//!************************************************************************
//! Get the CAN bus status
//!
//! @returns: nothing
//!************************************************************************
void CanOpen::updateStatus()
{
    if( mCanDevice && mCanDevice->hasBusStatus() )
    {
        switch( mCanDevice->busStatus() )
        {
            case QCanBusDevice::CanBusStatus::Good:
                mStatus = CAN_BUS_STATUS_OK;
                break;

            case QCanBusDevice::CanBusStatus::Warning:
                mStatus = CAN_BUS_STATUS_WARNING;
                break;

            case QCanBusDevice::CanBusStatus::Error:
                mStatus = CAN_BUS_STATUS_ERROR;
                break;

            case QCanBusDevice::CanBusStatus::BusOff:
                mStatus = CAN_BUS_STATUS_OFF;
                break;

            default:
                mStatus = CAN_BUS_STATUS_UNKNOWN;
                break;
        }
    }
    else
    {
        mStatus = CAN_BUS_STATUS_UNKNOWN;
        mStatusTimer->stop();
    }

    if( mIsConnected && ( CAN_BUS_STATUS_UNKNOWN == mStatus ) )
    {
        mStatusTimer->stop();
        mIsConnected = false;
        mConnectionStr = "CAN bus disconnected.";
        emit connectionStatusChanged( mIsConnected );
        emit connectionStrChanged( mConnectionStr );
    }

    mStatusStr = "CAN bus status: " + QString::fromStdString( CAN_BUS_STATUS_NAMES.at( mStatus ) );
    emit statusStrChanged( mStatusStr );
}
