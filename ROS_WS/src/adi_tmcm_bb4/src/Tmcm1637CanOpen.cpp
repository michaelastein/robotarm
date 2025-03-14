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
Tmcm1637CanOpen.cpp

This file contains the sources for TMCM-1637 (CANopen).
*/

#include <adi_tmcm_bb4/Tmcm1637CanOpen.hpp>

#include <QByteArray>
#include <QCanBusFrame>
#include <QThread>


const std::map<Tmcm1637CanOpen::MotionControllerState, std::string> Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_NAMES =
{
    { Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_UNKNOWN,            "Unknown"            },
    { Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_SWITCH_ON_DISABLED, "Switch on disabled" },
    { Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_READY_TO_SWITCH_ON, "Ready to switch on" },
    { Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_SWITCHED_ON,        "Switched on"        },
    { Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_OPERATION_ENABLED,  "Operation enabled"  },
    { Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_FAULT,              "Fault"              },
    { Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_QUICK_STOP,         "Quick stop"         }
};

const std::map<Tmcm1637CanOpen::OperatingMode, std::string> Tmcm1637CanOpen::OPERATING_MODE_NAMES =
{
    { Tmcm1637CanOpen::OPERATING_MODE_NONE,             "None"      },
    { Tmcm1637CanOpen::OPERATING_MODE_PROFILE_POSITION, "Position"  },
    { Tmcm1637CanOpen::OPERATING_MODE_PROFILE_VELOCITY, "Velocity"  },
    { Tmcm1637CanOpen::OPERATING_MODE_HOMING,           "Homing"    }
};

//!************************************************************************
//! Constructor
//!************************************************************************
Tmcm1637CanOpen::Tmcm1637CanOpen
    (
    uint16_t aNodeId    //!< CAN node ID
    )
    : mNodeId( aNodeId )
    , mTxFrameId( mNodeId + CanOpen::CAN_ID_BASE_TX )
    , mCommutationMode( COMMUTATION_MODE_DIGITAL_HALL )
    , mMotorType( MOTOR_TYPE_THREE_PHASE_BLDC )
    , mMaxCurrentMa( 1790 )
    , mPolePairs( 8 )
    , mShaftDirection( MOTOR_SHAFT_DIRECTION_NORMAL )
    , mProfileAccel( 2000 )
    , mProfileDecel( 2000 )
    , mAreParametersSet( false )
    , mControlWord( 0 )
    , mStatusWord( 0 )
    , mState( MOTION_CONTROLLER_STATE_UNKNOWN )
{
    memset( &mFwVer, 0, sizeof( mFwVer ) );
    memset( &mHallSettings, 0, sizeof( mHallSettings ) );
    mHallSettings.HallInterpolation = true;

    mCanOpen = CanOpen::getInstance();
}


//!************************************************************************
//! Destructor
//!************************************************************************
Tmcm1637CanOpen::~Tmcm1637CanOpen()
{
}


//!************************************************************************
//! Get the status if motor parameters are set
//!
//! @returns: true if the parameters are set
//!************************************************************************
bool Tmcm1637CanOpen::areMotorParametersSet() const
{
    return mAreParametersSet;
}


//!************************************************************************
//! Clear the status of motor parameters being set
//!
//! @returns: nothing
//!************************************************************************
void Tmcm1637CanOpen::clearMotorParametersStatus()
{
    mAreParametersSet = false;
}


//!************************************************************************
//! Convert a position value to the equivalent angle [degrees]
//! A full rotation is encoded to 32768 units.
//! For a 3-phase BLDC with 8 poles:
//! - the encoder increment is 32768/24 = 1365.33 units
//! - the angle increment is 360/24 = 15 degrees
//!
//! @returns: the angle corresponding to the position
//!************************************************************************
double Tmcm1637CanOpen::convertPositionToAngle
    (
    const int32_t aPosition //!< encoded position
    ) const
{
    const double RATIO = 360.0 / FULL_ROTATION_ENCODED;
    double angleDeg = aPosition * RATIO;
    return angleDeg;
}


//!************************************************************************
//! Get the CiA-402 control word
//!
//! @returns: true if the control word can be read
//!************************************************************************
bool Tmcm1637CanOpen::getControlWord
    (
    uint16_t& aControlWord  //!< CiA-402 control word
    )
{
    uint8_t length = 0;
    uint32_t data = 0;
    bool status = mCanOpen->readFrame( mTxFrameId, OBJECT_CONTROL_WORD, 0, length, data );

    if( status )
    {
        status = ( sizeof( uint16_t ) == length );
    }

    if( status )
    {
        aControlWord = static_cast<uint16_t>( data );
        mControlWord = aControlWord;
    }

    return status;
}


//!************************************************************************
//! Get the current position [-]
//!
//! @returns: true if the value can be read
//!************************************************************************
bool Tmcm1637CanOpen::getCurrentPosition
    (
    int32_t& aPosition  //!< position [-]
    ) const
{
    uint8_t length = 0;
    uint32_t data = 0;
    bool status = mCanOpen->readFrame( mTxFrameId, OBJECT_POSITION_ACTUAL_VALUE, 0, length, data );

    if( status )
    {
        status = ( sizeof( uint32_t ) == length );
    }

    if( status )
    {
        aPosition = static_cast<int32_t>( data );
    }

    return status;
}


//!************************************************************************
//! Get the current rotational speed [rpm]
//!
//! @returns: true if the value can be read
//!************************************************************************
bool Tmcm1637CanOpen::getCurrentVelocity
    (
    int32_t& aVelocity  //!< rotational speed [rpm]
    ) const
{
    uint8_t length = 0;
    uint32_t data = 0;
    bool status = mCanOpen->readFrame( mTxFrameId, OBJECT_VELOCITY_ACTUAL_VALUE, 0, length, data );

    if( status )
    {
        status = ( sizeof( uint32_t ) == length );
    }

    if( status )
    {
        aVelocity = static_cast<int32_t>( data );
    }

    return status;
}


//!************************************************************************
//! Read the motor status flags
//!
//! @returns: true if the motor status flags can be read
//!************************************************************************
bool Tmcm1637CanOpen::getMotorStatus
    (
    uint32_t& aMotorStatus  //!< motor status
    ) const
{
    uint8_t length = 0;
    uint32_t data = 0;
    bool status = mCanOpen->readFrame( mTxFrameId, OBJECT_MOTOR_STATUS_FLAGS, 0, length, data );

    if( status )
    {
        status = ( sizeof( uint32_t ) == length );
    }

    if( status )
    {
        aMotorStatus = data;
    }

    return status;
}


//!************************************************************************
//! Get the encoded position for a target position
//! The returned value depends on the number of phases, poles, and units
//! corresponding to 360 degs.
//!
//! @returns: the nearest encoded position
//!************************************************************************
int32_t Tmcm1637CanOpen::getEncodedTargetPosition
    (
    const int32_t aTargetPosition   //!< target position
    ) const
{
    const double ENCODER_INCREMENT = static_cast<double>( FULL_ROTATION_ENCODED ) / ( 3 * mPolePairs );
    int32_t q = aTargetPosition / ENCODER_INCREMENT;
    int32_t retVal = q * ENCODER_INCREMENT;
    return retVal;
}


//!************************************************************************
//! Get the CAN node ID of the board
//!
//! @returns: the CAN node ID
//!************************************************************************
uint16_t Tmcm1637CanOpen::getNodeId() const
{
    return mNodeId;
}


//!************************************************************************
//! Get the operating mode
//!
//! @returns: true if the mode can be read
//!************************************************************************
bool Tmcm1637CanOpen::getOperatingMode
    (
    OperatingMode& aMode   //!< operating mode
    ) const
{
    aMode = OPERATING_MODE_NONE;
    uint8_t length = 0;
    uint32_t data = 0;
    bool status = mCanOpen->readFrame( mTxFrameId, OBJECT_MODES_OF_OPERATION, 0, length, data );

    if( status )
    {
        status = ( sizeof( int8_t ) == length );
    }

    if( status )
    {
        aMode = static_cast<OperatingMode>( data );
    }

    return status;
}


//!************************************************************************
//! Get the controller state (CiA-402)
//!
//! @returns: true if the controller state can be read
//!************************************************************************
bool Tmcm1637CanOpen::getState
    (
    Tmcm1637CanOpen::MotionControllerState& aState //!< controller state
    )
{
    aState = MOTION_CONTROLLER_STATE_UNKNOWN;
    uint16_t statusWord = 0;
    bool status = getStatusWord( statusWord );

    if( status )
    {
        bool switchOnDisabled = statusWord & STATE_MACHINE_SWITCH_ON_DISABLED;
        bool readyToSwitchOn = statusWord & STATE_MACHINE_READY_TO_SWITCH_ON;
        bool switchedOn = statusWord & STATE_MACHINE_SWITCHED_ON;
        bool operationEnabled = statusWord & STATE_MACHINE_OPERATION_ENABLED;
        bool fault = statusWord & STATE_MACHINE_FAULT;

        if( fault )
        {
            aState = MOTION_CONTROLLER_STATE_FAULT;
        }
        else if( switchOnDisabled )
        {
            aState = MOTION_CONTROLLER_STATE_SWITCH_ON_DISABLED;
        }
        else if( readyToSwitchOn && switchedOn && operationEnabled )
        {
            aState = MOTION_CONTROLLER_STATE_OPERATION_ENABLED;
        }
        else if( readyToSwitchOn && switchedOn )
        {
            aState = MOTION_CONTROLLER_STATE_SWITCHED_ON;
        }
        else if( readyToSwitchOn )
        {
            aState = MOTION_CONTROLLER_STATE_READY_TO_SWITCH_ON;
        }
    }

    mState = aState;
    return status;
}


//!************************************************************************
//! Get the CiA-402 state machine status word
//!
//! @returns: true if the status word can be read
//!************************************************************************
bool Tmcm1637CanOpen::getStatusWord
    (
    uint16_t&   aStatusWord    //!< CiA-402 state machine status
    )
{
    uint8_t length = 0;
    uint32_t data = 0;
    bool status = mCanOpen->readFrame( mTxFrameId, OBJECT_STATUS_WORD, 0, length, data );

    if( status )
    {
        status = ( sizeof( uint16_t ) == length );
    }

    if( status )
    {
        aStatusWord = static_cast<uint16_t>( data );
        mStatusWord = aStatusWord;
    }

    return status;
}


//!************************************************************************
//! Get the target rotational speed [rpm]
//!
//! @returns: true if the value can be read
//!************************************************************************
bool Tmcm1637CanOpen::getTargetVelocity
    (
    int32_t& aVelocity  //!< rotational speed [rpm]
    ) const
{
    uint8_t length = 0;
    uint32_t data = 0;
    bool status = mCanOpen->readFrame( mTxFrameId, OBJECT_TARGET_VELOCITY, 0, length, data );

    if( status )
    {
        status = ( sizeof( uint32_t ) == length );
    }

    if( status )
    {
        aVelocity = static_cast<int32_t>( data );
    }

    return status;
}


//!************************************************************************
//! Get the Tx frame ID (COB-ID) of the board
//!
//! @returns: the Tx frame ID of the board
//!************************************************************************
uint16_t Tmcm1637CanOpen::getTxFrameId() const
{
    return mTxFrameId;
}


//!************************************************************************
//! Check if the vendor ID is correct
//!
//! @returns: true if the identity is correct
//!************************************************************************
bool Tmcm1637CanOpen::isIdentityCorrect()
{
    uint8_t length = 0;
    uint32_t data = 0;
    bool status = mCanOpen->readFrame( mTxFrameId, OBJECT_IDENTITY, 1, length, data );

    if( status )
    {
        const uint32_t VENDOR_ID = 0x286; // Trinamic
        status = ( sizeof( uint32_t ) == length ) && ( VENDOR_ID == data );
    }

    return status;
}


//!************************************************************************
//! Reset the Hall sensors direction.
//! Can be used for clearing bit 14 in StatusWord (motor activity).
//!
//! @returns: true if direction reset can be done
//!************************************************************************
bool Tmcm1637CanOpen::resetHallDirection()
{
    uint8_t length = 0;
    uint32_t data = 0;
    uint8_t currentHallDirection = 0;
    bool status = mCanOpen->readFrame( mTxFrameId, OBJECT_HALL_SENSOR_SETTINGS, 2, length, data );

    if( status )
    {
        status = ( sizeof( int8_t ) == length );
    }

    if( status )
    {
        currentHallDirection = static_cast<uint8_t>( data );
        status = mCanOpen->createAndSendFrame( mTxFrameId, OBJECT_HALL_SENSOR_SETTINGS, 2, static_cast<uint8_t>( !currentHallDirection ) );
    }

    const unsigned long DELAY_HALL_TOGGLE_US = 10;

    if( status )
    {
        QThread::usleep( DELAY_HALL_TOGGLE_US );
        status = mCanOpen->createAndSendFrame( mTxFrameId, OBJECT_HALL_SENSOR_SETTINGS, 2, currentHallDirection );
    }

    if( status )
    {
        QThread::usleep( DELAY_HALL_TOGGLE_US );
    }

    return status;
}


//!************************************************************************
//! Set the CiA-402 control word
//!
//! @returns: true if the control word can be written
//!************************************************************************
bool Tmcm1637CanOpen::setControlWord
    (
    const uint16_t aControlWord  //!< CiA-402 control word
    )
{
    bool status = mCanOpen->createAndSendFrame( mTxFrameId, OBJECT_CONTROL_WORD, 0, aControlWord );

    if( status )
    {
        mControlWord = aControlWord;
    }

    return status;
}


//!************************************************************************
//! Set the motor parameters
//!
//! @returns: true if all the parameters can be set
//!************************************************************************
bool Tmcm1637CanOpen::setMotorParameters()
{
    bool status = mCanOpen->createAndSendFrame( mTxFrameId, OBJECT_COMMUTATION_MODE, 0, mCommutationMode );

    if( status )
    {
        status = mCanOpen->createAndSendFrame( mTxFrameId, OBJECT_MOTOR_TYPE, 0, mMotorType );
    }

    if( status )
    {
        status = mCanOpen->createAndSendFrame( mTxFrameId, OBJECT_MAXIMUM_CURRENT, 0, mMaxCurrentMa );
    }

    if( status )
    {
        status = mCanOpen->createAndSendFrame( mTxFrameId, OBJECT_MOTOR_POLE_PAIRS, 0, mPolePairs );
    }

    if( status )
    {
        status = mCanOpen->createAndSendFrame( mTxFrameId, OBJECT_HALL_SENSOR_SETTINGS, 1, static_cast<uint8_t>( mHallSettings.HallPolarity ) );
    }

    if( status )
    {
        status = mCanOpen->createAndSendFrame( mTxFrameId, OBJECT_HALL_SENSOR_SETTINGS, 2, static_cast<uint8_t>( mHallSettings.HallDirection ) );
    }

    if( status )
    {
        status = mCanOpen->createAndSendFrame( mTxFrameId, OBJECT_HALL_SENSOR_SETTINGS, 3, static_cast<uint8_t>( mHallSettings.HallInterpolation ) );
    }

    if( status )
    {
        status = mCanOpen->createAndSendFrame( mTxFrameId, OBJECT_HALL_SENSOR_SETTINGS, 4, static_cast<int16_t>( mHallSettings.HallPhieOffset ) );
    }

    if( status )
    {
        status = mCanOpen->createAndSendFrame( mTxFrameId, OBJECT_MOTOR_SHAFT_DIRECTION, 0, mShaftDirection );
    }

    if( status )
    {
        status = mCanOpen->createAndSendFrame( mTxFrameId, OBJECT_PROFILE_ACCELERATION, 0, mProfileAccel );
    }

    if( status )
    {
        status = mCanOpen->createAndSendFrame( mTxFrameId, OBJECT_PROFILE_DECELERATION, 0, mProfileDecel );
    }

    if( status )
    {
        // applies to position mode only
        status = mCanOpen->createAndSendFrame( mTxFrameId, OBJECT_PROFILE_VELOCITY, 0, POSITION_MODE_VELOCITY_RPM );
    }

    mAreParametersSet = status;
    return status;
}


//!************************************************************************
//! Set the CAN node ID
//!
//! @returns: nothing
//!************************************************************************
void Tmcm1637CanOpen::setNodeId
    (
    const uint16_t aNodeId   //!< CAN node ID
    )
{
    mNodeId = aNodeId;
    mTxFrameId = mNodeId + CanOpen::CAN_ID_BASE_TX;
}


//!************************************************************************
//! Set the operating mode
//! Have to read first the existing mode because it is not allowed to write
//! the same value twice.
//!
//! @returns: true if the mode can be set or is already the same
//!************************************************************************
bool Tmcm1637CanOpen::setOperatingMode
    (
    const OperatingMode aMode       //!< operating mode
    ) const
{
    uint8_t length = 0;
    uint32_t data = 0;
    bool status = false;

    switch( aMode )
    {
        case OPERATING_MODE_NONE:
        case OPERATING_MODE_PROFILE_POSITION:
        case OPERATING_MODE_PROFILE_VELOCITY:
        case OPERATING_MODE_HOMING:
            status = true;
            break;

        default:
            break;
    }

    if( status )
    {
        status = mCanOpen->readFrame( mTxFrameId, OBJECT_MODES_OF_OPERATION, 0, length, data );
    }

    if( status )
    {
        status = ( sizeof( int8_t ) == length );
    }

    if( status )
    {
        OperatingMode crtMode = static_cast<OperatingMode>( data );

        if( aMode != crtMode )
        {
            status = mCanOpen->createAndSendFrame( mTxFrameId, OBJECT_MODES_OF_OPERATION, 0, static_cast<int8_t>( aMode ) );
        }
    }

    return status;
}


//!************************************************************************
//! Set the velocity [rpm] for position mode
//!
//! @returns: true if the velocity can be set
//!************************************************************************
bool Tmcm1637CanOpen::setPositionModeVelocity
    (
    const uint32_t aVelocity //!< rotational speed [rpm]
    )
{
    bool status = mCanOpen->createAndSendFrame( mTxFrameId, OBJECT_PROFILE_VELOCITY, 0, aVelocity );
    return status;
}


//!************************************************************************
//! Set the controller state (CiA-402)
//!
//! @returns: true if the new state can be set
//!************************************************************************
bool Tmcm1637CanOpen::setState
    (
    const MotionControllerState aState  //!< motion controller state
    )
{
    uint16_t controlWord = 0;
    uint16_t statusWord = 0;
    bool status = getStatusWord( statusWord );

    if( status )
    {
        bool fault = statusWord & STATE_MACHINE_FAULT;

        if( fault )
        {
            status = getControlWord( controlWord );

            if( status )
            {
                controlWord |= CONTROL_WORD_FAULT_RESET;
                status = setControlWord( controlWord );
            }

            if( status )
            {
                status = getStatusWord( statusWord );
            }

            if( status )
            {
                fault = statusWord & STATE_MACHINE_FAULT;

                if( fault )
                {
                    status = false;
                }
            }
        }
    }

    if( status )
    {
        bool switchOnDisabled = statusWord & STATE_MACHINE_SWITCH_ON_DISABLED;
        bool readyToSwitchOn = statusWord & STATE_MACHINE_READY_TO_SWITCH_ON;
        bool switchedOn = statusWord & STATE_MACHINE_SWITCHED_ON;
        bool operationEnabled = statusWord & STATE_MACHINE_OPERATION_ENABLED;
        bool motorActive = statusWord & STATE_MACHINE_MOTOR_ACTIVITY;

        switch( aState )
        {
            case MOTION_CONTROLLER_STATE_SWITCH_ON_DISABLED:
                if( readyToSwitchOn || switchedOn || operationEnabled )
                {
                    controlWord = CONTROL_WORD_SWITCH_ON
                                | CONTROL_WORD_QUICK_STOP_ACTIVE_LOW;
                    status = setControlWord( controlWord ); // 0x0005

                    if( operationEnabled )
                    {
                        if( status )
                        {
                            const unsigned long DELAY_HALL_SWITCH_ON_DISABLED_US = 1000;
                            QThread::usleep( DELAY_HALL_SWITCH_ON_DISABLED_US );

                            if( getStatusWord( statusWord ) )
                            {
                                motorActive = statusWord & STATE_MACHINE_MOTOR_ACTIVITY;

                                if( motorActive )
                                {
                                    resetHallDirection();
                                }
                            }
                        }
                    }
                }
                break;

            case MOTION_CONTROLLER_STATE_READY_TO_SWITCH_ON:
                if( switchOnDisabled || switchedOn || operationEnabled )
                {
                    controlWord = CONTROL_WORD_ENABLE_VOLTAGE
                                | CONTROL_WORD_QUICK_STOP_ACTIVE_LOW;
                    status = setControlWord( controlWord ); // 0x0006

                    if( operationEnabled )
                    {
                        if( status )
                        {
                            const unsigned long DELAY_HALL_READY_TO_SWITCH_ON_US = 10;
                            QThread::usleep( DELAY_HALL_READY_TO_SWITCH_ON_US );

                            if( getStatusWord( statusWord ) )
                            {
                                motorActive = statusWord & STATE_MACHINE_MOTOR_ACTIVITY;

                                if( motorActive )
                                {
                                    resetHallDirection();
                                }
                            }
                        }
                    }
                }
                break;

            case MOTION_CONTROLLER_STATE_SWITCHED_ON:
                if( readyToSwitchOn || operationEnabled )
                {
                    controlWord = CONTROL_WORD_SWITCH_ON
                                | CONTROL_WORD_ENABLE_VOLTAGE
                                | CONTROL_WORD_QUICK_STOP_ACTIVE_LOW;
                    status = setControlWord( controlWord ); // 0x0007

                    if( operationEnabled )
                    {
                        if( status )
                        {
                            const unsigned long DELAY_HALL_SWITCHED_ON_US = 10;
                            QThread::usleep( DELAY_HALL_SWITCHED_ON_US );

                            if( getStatusWord( statusWord ) )
                            {
                                motorActive = statusWord & STATE_MACHINE_MOTOR_ACTIVITY;

                                if( motorActive )
                                {
                                    resetHallDirection();
                                }
                            }
                        }
                    }
                }
                else if( switchOnDisabled )
                {
                    // 1st step -> MOTION_CONTROLLER_STATE_READY_TO_SWITCH_ON
                    controlWord = CONTROL_WORD_ENABLE_VOLTAGE
                                | CONTROL_WORD_QUICK_STOP_ACTIVE_LOW;
                    status = setControlWord( controlWord ); // 0x0006

                    // 2nd step -> MOTION_CONTROLLER_STATE_SWITCHED_ON
                    if( status )
                    {
                        controlWord = CONTROL_WORD_SWITCH_ON
                                    | CONTROL_WORD_ENABLE_VOLTAGE
                                    | CONTROL_WORD_QUICK_STOP_ACTIVE_LOW;
                        status = setControlWord( controlWord ); // 0x0007
                    }
                }
                break;

            case MOTION_CONTROLLER_STATE_OPERATION_ENABLED:
                if( switchedOn )
                {
                    controlWord = CONTROL_WORD_SWITCH_ON
                                | CONTROL_WORD_ENABLE_VOLTAGE
                                | CONTROL_WORD_QUICK_STOP_ACTIVE_LOW
                                | CONTROL_WORD_ENABLE_OPERATION;
                    status = setControlWord( controlWord ); // 0x000F
                }
                else if( readyToSwitchOn )
                {
                    // 1st step -> MOTION_CONTROLLER_STATE_SWITCHED_ON
                    controlWord = CONTROL_WORD_SWITCH_ON
                                | CONTROL_WORD_ENABLE_VOLTAGE
                                | CONTROL_WORD_QUICK_STOP_ACTIVE_LOW;
                    status = setControlWord( controlWord ); // 0x0007

                    // 2nd step -> MOTION_CONTROLLER_STATE_OPERATION_ENABLED
                    if( status )
                    {
                        controlWord = CONTROL_WORD_SWITCH_ON
                                    | CONTROL_WORD_ENABLE_VOLTAGE
                                    | CONTROL_WORD_QUICK_STOP_ACTIVE_LOW
                                    | CONTROL_WORD_ENABLE_OPERATION;
                        status = setControlWord( controlWord ); // 0x000F
                    }
                }
                else if( switchOnDisabled )
                {
                    // 1st step -> MOTION_CONTROLLER_STATE_READY_TO_SWITCH_ON
                    controlWord = CONTROL_WORD_ENABLE_VOLTAGE
                                | CONTROL_WORD_QUICK_STOP_ACTIVE_LOW;
                    status = setControlWord( controlWord ); // 0x0006

                    // 2nd step -> MOTION_CONTROLLER_STATE_SWITCHED_ON
                    if( status )
                    {
                        controlWord = CONTROL_WORD_SWITCH_ON
                                    | CONTROL_WORD_ENABLE_VOLTAGE
                                    | CONTROL_WORD_QUICK_STOP_ACTIVE_LOW;
                        status = setControlWord( controlWord ); // 0x0007
                    }

                    // 3rd step -> MOTION_CONTROLLER_STATE_OPERATION_ENABLED
                    if( status )
                    {
                        controlWord = CONTROL_WORD_SWITCH_ON
                                    | CONTROL_WORD_ENABLE_VOLTAGE
                                    | CONTROL_WORD_QUICK_STOP_ACTIVE_LOW
                                    | CONTROL_WORD_ENABLE_OPERATION;
                        status = setControlWord( controlWord ); // 0x000F
                    }
                }
                break;

            case MOTION_CONTROLLER_STATE_QUICK_STOP:
                if( operationEnabled )
                {
                    controlWord = CONTROL_WORD_ENABLE_VOLTAGE;
                    status = setControlWord( controlWord ); // 0x0002
                }
                break;

            default:
                break;
        }
    }

    return status;
}


//!************************************************************************
//! Set the target position [-]
//!
//! @returns: true if the position can be set
//!************************************************************************
bool Tmcm1637CanOpen::setTargetPosition
    (
    const int32_t aPosition //!< position [-]
    )
{
    bool status = mCanOpen->createAndSendFrame( mTxFrameId, OBJECT_TARGET_POSITION, 0, aPosition );
    return status;
}


//!************************************************************************
//! Set the target velocity [rpm]
//!
//! @returns: true if the velocity can be set
//!************************************************************************
bool Tmcm1637CanOpen::setTargetVelocity
    (
    const int32_t aVelocity //!< rotational speed [rpm]
    )
{
    bool status = mCanOpen->createAndSendFrame( mTxFrameId, OBJECT_TARGET_VELOCITY, 0, aVelocity );
    return status;
}
