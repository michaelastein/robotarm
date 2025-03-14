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
Tmcm1637CanOpen.h

This file contains the definitions for TMCM-1637 (CANopen).
*/

#ifndef Tmcm1637CanOpen_h
#define Tmcm1637CanOpen_h

#include "CanOpen.h"

#include <cstdint>
#include <map>
#include <string>
#include <vector>


//************************************************************************
// Class for handling the TMCM-1637 board (CANopen)
//************************************************************************
class Tmcm1637CanOpen
{
    //************************************************************************
    // constants and types
    //************************************************************************
    public:
        typedef enum : uint16_t
        {
            //**********************************
            // Communication area
            //**********************************
            OBJECT_IDENTITY                 = 0x1018,

            //**********************************
            // Manufacturer specific area
            //**********************************
            OBJECT_MAXIMUM_CURRENT          = 0x2003,
            OBJECT_MOTOR_TYPE               = 0x2050,
            OBJECT_COMMUTATION_MODE         = 0x2055,
            OBJECT_MOTOR_POLE_PAIRS         = 0x2056,
            OBJECT_MOTOR_SHAFT_DIRECTION    = 0x2057,
            OBJECT_HALL_SENSOR_SETTINGS     = 0x2070,
            OBJECT_MOTOR_STATUS_FLAGS       = 0x2101,

            //**********************************
            // Profile specific area
            //**********************************
            OBJECT_CONTROL_WORD             = 0x6040,
            OBJECT_STATUS_WORD              = 0x6041,
            OBJECT_MODES_OF_OPERATION       = 0x6060,

            //**********************************
            // Profile position mode
            //**********************************
            OBJECT_POSITION_ACTUAL_VALUE    = 0x6064,
            OBJECT_TARGET_POSITION          = 0x607A,
            OBJECT_PROFILE_VELOCITY         = 0x6081,

            //**********************************
            // Profile velocity mode
            //**********************************
            OBJECT_VELOCITY_ACTUAL_VALUE    = 0x606C,
            OBJECT_PROFILE_ACCELERATION     = 0x6083,
            OBJECT_PROFILE_DECELERATION     = 0x6084,
            OBJECT_TARGET_VELOCITY          = 0x60FF
        }Object;

        typedef enum : uint8_t
        {
            MOTION_CONTROLLER_STATE_UNKNOWN,

            MOTION_CONTROLLER_STATE_SWITCH_ON_DISABLED,
            MOTION_CONTROLLER_STATE_READY_TO_SWITCH_ON,
            MOTION_CONTROLLER_STATE_SWITCHED_ON,
            MOTION_CONTROLLER_STATE_OPERATION_ENABLED,
            MOTION_CONTROLLER_STATE_FAULT,
            MOTION_CONTROLLER_STATE_QUICK_STOP
        }MotionControllerState;

        static const std::map<MotionControllerState, std::string> MOTION_CONTROLLER_STATE_NAMES;

        typedef enum : uint8_t
        {
            COMMUTATION_MODE_DISABLED     = 0,
            COMMUTATION_MODE_OPEN_LOOP    = 1,
            COMMUTATION_MODE_DIGITAL_HALL = 2,
            COMMUTATION_MODE_ABN_ENCODER  = 3
        }CommutationMode;

        typedef enum : uint8_t
        {
            MOTOR_TYPE_NONE             = 0,
            MOTOR_TYPE_SINGLE_PHASE_DC  = 1,
            MOTOR_TYPE_THREE_PHASE_BLDC = 3
        }MotorType;

        typedef enum : uint8_t
        {
            MOTOR_SHAFT_DIRECTION_NORMAL    = 0,
            MOTOR_SHAFT_DIRECTION_REVERSED  = 1
        }MotorShaftDirection;

        typedef enum : uint32_t
        {
            MOTOR_STATUS_OVERCURRENT     = 1,
            MOTOR_STATUS_UNDERVOLTAGE    = 1 << 1,
            MOTOR_STATUS_OVERVOLTAGE     = 1 << 2,
            MOTOR_STATUS_OVERTEMPERATURE = 1 << 3,
            MOTOR_STATUS_HALTED          = 1 << 4,
            MOTOR_STATUS_HALL_ERROR      = 1 << 5,
            MOTOR_STATUS_DRIVER_ERROR    = 1 << 6,
            MOTOR_STATUS_INIT_ERROR      = 1 << 7,
            MOTOR_STATUS_STOP_MODE       = 1 << 8,
            MOTOR_STATUS_VELOCITY_MODE   = 1 << 9,
            MOTOR_STATUS_POSITION_MODE   = 1 << 10,
            MOTOR_STATUS_TORQUE_MODE     = 1 << 11,
            MOTOR_STATUS_EMERGENCY_STOP  = 1 << 12,
            MOTOR_STATUS_POSITION_END    = 1 << 14,
            MOTOR_STATUS_MODULE_INIT     = 1 << 15,
            MOTOR_STATUS_IIT_EXCEEDED    = 1 << 17,
            MOTOR_STATUS_BRAKE_ACTIVE    = 1 << 18
        }MotorStatus;

        typedef enum : uint16_t
        {
            CONTROL_WORD_SWITCH_ON             = 1,
            CONTROL_WORD_ENABLE_VOLTAGE        = 1 << 1,
            CONTROL_WORD_QUICK_STOP_ACTIVE_LOW = 1 << 2,
            CONTROL_WORD_ENABLE_OPERATION      = 1 << 3,
            CONTROL_WORD_NEW_SET_POINT         = 1 << 4,
            CONTROL_WORD_NEW_POSITION_RELATIVE = 1 << 6,
            CONTROL_WORD_FAULT_RESET           = 1 << 7,
            CONTROL_WORD_HALT                  = 1 << 8
        }ControlWord; // CiA-402 control word

        typedef enum : uint16_t
        {
            STATE_MACHINE_READY_TO_SWITCH_ON    = 1,
            STATE_MACHINE_SWITCHED_ON           = 1 << 1,
            STATE_MACHINE_OPERATION_ENABLED     = 1 << 2,
            STATE_MACHINE_FAULT                 = 1 << 3,
            STATE_MACHINE_VOLTAGE_ENABLED       = 1 << 4,
            STATE_MACHINE_QUICK_STOP            = 1 << 5,
            STATE_MACHINE_SWITCH_ON_DISABLED    = 1 << 6,
            STATE_MACHINE_WARNING               = 1 << 7,
            STATE_MACHINE_MANUFACTURER_SPECIFIC = 1 << 8,
            STATE_MACHINE_REMOTE                = 1 << 9,
            STATE_MACHINE_TARGET_REACHED        = 1 << 10,
            STATE_MACHINE_INTERNAL_LIMIT_ACTIVE = 1 << 11,
            STATE_MACHINE_SETPOINT_ACKNOWLEDGE  = 1 << 12,
            STATE_MACHINE_FOLLOWING_ERROR       = 1 << 13,
            STATE_MACHINE_MOTOR_ACTIVITY        = 1 << 14,
            STATE_MACHINE_SHAFT_DIRECTION       = 1 << 15
        }StateMachine; // CiA-402 status word

        typedef enum : int8_t
        {
            OPERATING_MODE_NONE             = 0,
            OPERATING_MODE_PROFILE_POSITION = 1,
            OPERATING_MODE_PROFILE_VELOCITY = 3,
            OPERATING_MODE_HOMING           = 6,
            /*
            OPERATING_MODE_CYC_SYNC_POSITION= 8,
            OPERATING_MODE_CYC_SYNC_VELOCITY= 9,
            OPERATING_MODE_CYC_SYNC_TORQUE  = 10
            */
        }OperatingMode;

        static const std::map<OperatingMode, std::string> OPERATING_MODE_NAMES;

        typedef struct
        {
            MotionControllerState               currentState;
            std::vector<MotionControllerState>  nextStates;
        }AllowedStateTransitions;

        static const int32_t MAX_VELOCITY_RPM = 2000;   //!< [-2000..2000]

    private:
        typedef struct
        {
            bool    HallPolarity;
            bool    HallDirection;
            bool    HallInterpolation;
            int16_t HallPhieOffset;
        }HallSettings;

        typedef struct
        {
            uint16_t major;
            uint16_t minor;
        }Version;

        static const uint16_t FULL_ROTATION_ENCODED = 32768;    //!< units for 360 degs

        static const uint32_t POSITION_MODE_VELOCITY_RPM = 120; //!< used only in position mode (obj 0x6081)


    //************************************************************************
    // functions
    //************************************************************************
    public:
        Tmcm1637CanOpen
            (
            uint16_t aNodeId = 0    //!< CAN node ID
            );

        ~Tmcm1637CanOpen();

        bool areMotorParametersSet() const;

        void clearMotorParametersStatus();

        double convertPositionToAngle
            (
            const int32_t aPosition //!< encoded position
            ) const;

        bool getControlWord
            (
            uint16_t& aControlWord  //!< CiA-402 control word
            );

        bool getCurrentPosition
            (
            int32_t& aPosition  //!< position [-]
            ) const;

        bool getCurrentVelocity
            (
            int32_t& aVelocity  //!< rotational speed [rpm]
            ) const;

        int32_t getEncodedTargetPosition
            (
            const int32_t aTargetPosition   //!< target position
            ) const;

        uint16_t getNodeId() const;

        uint16_t getTxFrameId() const;

        bool getMotorStatus
            (
            uint32_t& aMotorStatus   //!< motor status
            ) const;

        bool getOperatingMode
            (
            OperatingMode& aMode   //!< operating mode
            ) const;

        bool getState
            (
            MotionControllerState& aState //!< controller state
            );

        bool getStatusWord
            (
            uint16_t&   aStatusWord    //!< CiA-402 state machine status
            );

        bool getTargetVelocity
            (
            int32_t& aVelocity  //!< rotational speed [rpm]
            ) const;

        bool goToHomePosition();

        bool isIdentityCorrect();

        bool resetHallDirection();

        bool setControlWord
            (
            const uint16_t aControlWord  //!< CiA-402 control word
            );

        void setHomePosition
            (
            const int32_t aPosition //!< position
            );

        bool setMotorParameters();

        void setNodeId
            (
            const uint16_t aNodeId   //!< CAN node ID
            );

        bool setOperatingMode
            (
            const OperatingMode aMode   //!< operating mode
            ) const;

        bool setPositionModeVelocity
            (
            const uint32_t aVelocity //!< rotational speed [rpm]
            );

        bool setState
            (
            const MotionControllerState aState  //!< motion controller state
            );

        bool setTargetPosition
            (
            const int32_t aPosition //!< position [-]
            );

        bool setTargetVelocity
            (
            const int32_t aVelocity //!< rotational speed [rpm]
            );

    private:
        void defineAllowedStateTransitions();


    //************************************************************************
    // variables
    //************************************************************************
    private:
        CanOpen*            mCanOpen;           //!< CANopen instance

        uint16_t            mNodeId;            //!< CAN node ID
        uint16_t            mTxFrameId;         //!< command COB-ID
        Version             mFwVer;             //!< firmware version

        CommutationMode     mCommutationMode;   //!< motor commutation mode
        MotorType           mMotorType;         //!< motor type
        uint32_t            mMaxCurrentMa;      //!< motor max current [mA]
        uint8_t             mPolePairs;         //!< motor number of poles
        HallSettings        mHallSettings;      //!< Hall sensor settings
        MotorShaftDirection mShaftDirection;    //!< motor shaft direction
        uint32_t            mProfileAccel;      //!< acceleration [rpm/s]
        uint32_t            mProfileDecel;      //!< deceleration [rpm/s]
        bool                mAreParametersSet;  //!< if parameters are set

        uint16_t            mControlWord;       //!< CiA-402 control word
        uint16_t            mStatusWord;        //!< CiA-402 state machine status
        int32_t             mHomePosition;      //!< home position
        MotionControllerState                   mState;                     //!< controller state (CiA-402)
        std::vector<AllowedStateTransitions>    mAlowedStateTransitionsVec; //!< vector with allowed transitions
};

#endif // Tmcm1637CanOpen_h
