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
TmcmBb4.cpp

This file contains the sources for TMCM-BB4.
*/

#include "TmcmBb4.h"
#include "./ui_TmcmBb4.h"

#include <QDebug>
#include <QMessageBox>
#include <QTimer>


//!************************************************************************
//! Constructor
//!************************************************************************
TmcmBb4::TmcmBb4
    (
    QWidget* aParent    //!< parent widget
    )
    : QMainWindow( aParent )
    , mMainUi( new Ui::TmcmBb4 )
    , mStartAutoConnect( true )
    , mMotorsPollingTimer( new QTimer() )
{
    mMainUi->setupUi( this );

    mCanOpen = CanOpen::getInstance();

    // exit
    connect( mMainUi->ExitButton, SIGNAL( clicked() ), this, SLOT( handleExit() ) );

    // CANopen signals
    connect( mCanOpen, &CanOpen::connectionStatusChanged, this, &TmcmBb4::updateCanConnectStatus );
    connect( mCanOpen, &CanOpen::connectionStrChanged, this, &TmcmBb4::updateCanConnectionStr );
    connect( mCanOpen, &CanOpen::statusStrChanged, this, &TmcmBb4::updateCanStatusStr );

    // CAN buttons
    connect( mMainUi->CanConnectButton, SIGNAL( clicked() ), this, SLOT( handleCanConnect() ) );
    connect( mMainUi->CanDisconnectButton, SIGNAL( clicked() ), this, SLOT( handleCanDisconnect() ) );
    connect( mMainUi->CanResetButton, SIGNAL( clicked() ), this, SLOT( handleCanReset() ) );

    // CAN status
    mMainUi->CanConnectionLabel->setText( "" );
    mMainUi->CanStatusLabel->setText( "" );

    // motion controller boards
    mTmcm1637Vec.resize( TMCM1637_SLOTS );
    mTmcmIdentityVec.resize( TMCM1637_SLOTS );
    mTmcmStateVec.resize( TMCM1637_SLOTS );
    mTmcmOperEnVec.resize( TMCM1637_SLOTS );

    for( size_t i = 0; i < mTmcm1637Vec.size(); i++ )
    {
        mTmcm1637Vec.at( i ).setNodeId( i + 1 );
    }

    // CANopen connect and initialize
    if( mStartAutoConnect )
    {
        handleCanConnect();
    }

    // velocity spin boxes
    mMainUi->SetVelocity_0_Spin->setMinimum( -Tmcm1637CanOpen::MAX_VELOCITY_RPM );
    mMainUi->SetVelocity_0_Spin->setMaximum(  Tmcm1637CanOpen::MAX_VELOCITY_RPM );

    mMainUi->SetVelocity_1_Spin->setMinimum( -Tmcm1637CanOpen::MAX_VELOCITY_RPM );
    mMainUi->SetVelocity_1_Spin->setMaximum(  Tmcm1637CanOpen::MAX_VELOCITY_RPM );

    mMainUi->SetVelocity_2_Spin->setMinimum( -Tmcm1637CanOpen::MAX_VELOCITY_RPM );
    mMainUi->SetVelocity_2_Spin->setMaximum(  Tmcm1637CanOpen::MAX_VELOCITY_RPM );

    mMainUi->SetVelocity_3_Spin->setMinimum( -Tmcm1637CanOpen::MAX_VELOCITY_RPM );
    mMainUi->SetVelocity_3_Spin->setMaximum(  Tmcm1637CanOpen::MAX_VELOCITY_RPM );

    connect( mMainUi->SetVelocity_0_Spin, SIGNAL( valueChanged(int) ), this, SLOT( handleVelocity0(int) ) );
    connect( mMainUi->SetVelocity_1_Spin, SIGNAL( valueChanged(int) ), this, SLOT( handleVelocity1(int) ) );
    connect( mMainUi->SetVelocity_2_Spin, SIGNAL( valueChanged(int) ), this, SLOT( handleVelocity2(int) ) );
    connect( mMainUi->SetVelocity_3_Spin, SIGNAL( valueChanged(int) ), this, SLOT( handleVelocity3(int) ) );

    // position spin boxes
    connect( mMainUi->SetPosition_0_Spin, SIGNAL( valueChanged(int) ), this, SLOT( handlePosition0(int) ) );
    connect( mMainUi->SetPosition_1_Spin, SIGNAL( valueChanged(int) ), this, SLOT( handlePosition1(int) ) );
    connect( mMainUi->SetPosition_2_Spin, SIGNAL( valueChanged(int) ), this, SLOT( handlePosition2(int) ) );
    connect( mMainUi->SetPosition_3_Spin, SIGNAL( valueChanged(int) ), this, SLOT( handlePosition3(int) ) );

    // operating modes
    for( const auto& i : Tmcm1637CanOpen::OPERATING_MODE_NAMES )
    {
        mMainUi->OperatingMode_0_ComboBox->addItem( QString::fromStdString( i.second ), i.first );
        mMainUi->OperatingMode_1_ComboBox->addItem( QString::fromStdString( i.second ), i.first );
        mMainUi->OperatingMode_2_ComboBox->addItem( QString::fromStdString( i.second ), i.first );
        mMainUi->OperatingMode_3_ComboBox->addItem( QString::fromStdString( i.second ), i.first );
    }

    connect( mMainUi->OperatingMode_0_ComboBox, SIGNAL( currentIndexChanged(int) ), this, SLOT( handleOperatingMode0(int) ) );
    connect( mMainUi->OperatingMode_1_ComboBox, SIGNAL( currentIndexChanged(int) ), this, SLOT( handleOperatingMode1(int) ) );
    connect( mMainUi->OperatingMode_2_ComboBox, SIGNAL( currentIndexChanged(int) ), this, SLOT( handleOperatingMode2(int) ) );
    connect( mMainUi->OperatingMode_3_ComboBox, SIGNAL( currentIndexChanged(int) ), this, SLOT( handleOperatingMode3(int) ) );

    // operation enable/disable
    connect( mMainUi->Operation_0_Button, SIGNAL( clicked() ), this, SLOT( handleOperationEnable0() ) );
    connect( mMainUi->Operation_1_Button, SIGNAL( clicked() ), this, SLOT( handleOperationEnable1() ) );
    connect( mMainUi->Operation_2_Button, SIGNAL( clicked() ), this, SLOT( handleOperationEnable2() ) );
    connect( mMainUi->Operation_3_Button, SIGNAL( clicked() ), this, SLOT( handleOperationEnable3() ) );

    // home update
    connect( mMainUi->HomeUpdate_0_Button, SIGNAL( clicked() ), this, SLOT( handleHomeUpdate0() ) );
    connect( mMainUi->HomeUpdate_1_Button, SIGNAL( clicked() ), this, SLOT( handleHomeUpdate1() ) );
    connect( mMainUi->HomeUpdate_2_Button, SIGNAL( clicked() ), this, SLOT( handleHomeUpdate2() ) );
    connect( mMainUi->HomeUpdate_3_Button, SIGNAL( clicked() ), this, SLOT( handleHomeUpdate3() ) );

    // timer for polling the motion controllers
    connect( mMotorsPollingTimer, &QTimer::timeout, this, &TmcmBb4::pollMotors );
}


//!************************************************************************
//! Destructor
//!************************************************************************
TmcmBb4::~TmcmBb4()
{
    delete mMainUi;
}


//!************************************************************************
//! Check the motion controller boards
//!
//! @returns: nothing
//!************************************************************************
void TmcmBb4::checkMotionControllers()
{
    if( mCanOpen->isConnected() )
    {
        bool troubleFound = false;
        QString msg;

        //***********************************************************
        // init
        //***********************************************************
        for( size_t i = 0; i < mTmcm1637Vec.size(); i++ )
        {
            mTmcmIdentityVec.at( i ) = false;
            mTmcmStateVec.at( i ) = false;
            mTmcmOperEnVec.at( i ) = false;
            mTmcm1637Vec.at( i ).clearMotorParametersStatus();
        }

        //***********************************************************
        // presence check
        //***********************************************************
        troubleFound = false;

        for( size_t i = 0; i < mTmcm1637Vec.size(); i++ )
        {
            mTmcmIdentityVec.at( i ) = mTmcm1637Vec.at( i ).isIdentityCorrect();

            if( !mTmcmIdentityVec.at( i ) )
            {
                troubleFound = true;
            }
        }

        if( troubleFound )
        {
            msg = "Could not find TMCM-1637 modules with node ID: ";
            bool haveFirst = false;

            for( size_t i = 0; i < mTmcm1637Vec.size(); i++ )
            {
                if( !mTmcmIdentityVec.at( i ) )
                {
                    if( haveFirst )
                    {
                        msg += ", ";
                    }

                    msg += QString::number( mTmcm1637Vec.at( i ).getNodeId() );
                    haveFirst = true;
                }
            }

            QMessageBox::critical( this, "TMCM-BB4 CANopen", msg, QMessageBox::Ok );
        }

        //***********************************************************
        // set state to SWITCH_ON_DISABLED
        //***********************************************************
        troubleFound = false;

        for( size_t i = 0; i < mTmcm1637Vec.size(); i++ )
        {
            if( mTmcmIdentityVec.at( i ) )
            {
                Tmcm1637CanOpen::MotionControllerState state = Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_UNKNOWN;
                bool status = mTmcm1637Vec.at( i ).getState( state );

                if( status )
                {
                    mTmcmOperEnVec.at( i ) = ( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_OPERATION_ENABLED == state );

                    if( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_SWITCH_ON_DISABLED != state )
                    {
                        status = mTmcm1637Vec.at( i ).setState( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_SWITCH_ON_DISABLED );
                        mTmcmStateVec.at( i ) = status;

                        if( status )
                        {
                            mTmcmOperEnVec.at( i ) = false;
                        }
                        else
                        {
                            troubleFound = true;
                        }
                    }
                    else
                    {
                        mTmcmStateVec.at( i ) = true;
                    }
                }
                else
                {
                    mTmcmStateVec.at( i ) = false;
                    troubleFound = true;
                }                    
            }
        }

        if( troubleFound )
        {
            msg = "Could not set state to SWITCH_ON_DISABLED for TMCM-1637 modules\nwith node ID: ";
            bool haveFirst = false;

            for( size_t i = 0; i < mTmcm1637Vec.size(); i++ )
            {
                if( !mTmcmStateVec.at( i ) )
                {
                    if( haveFirst )
                    {
                        msg += ", ";
                    }

                    msg += QString::number( mTmcm1637Vec.at( i ).getNodeId() );
                    haveFirst = true;
                }
            }

            QMessageBox::critical( this, "TMCM-BB4 CANopen", msg, QMessageBox::Ok );
        }

        //***********************************************************
        // set motor parameters
        //***********************************************************
        troubleFound = false;

        for( size_t i = 0; i < mTmcm1637Vec.size(); i++ )
        {
            if( mTmcmIdentityVec.at( i ) && mTmcmStateVec.at( i ) )
            {                
                if( !mTmcm1637Vec.at( i ).setMotorParameters() )
                {
                    troubleFound = true;
                }
            }
        }

        if( troubleFound )
        {
            msg = "Could not set motor parameters for TMCM-1637 modules \nwith node ID: ";
            bool haveFirst = false;

            for( size_t i = 0; i < mTmcm1637Vec.size(); i++ )
            {
                if( !mTmcm1637Vec.at( i ).areMotorParametersSet() )
                {
                    if( haveFirst )
                    {
                        msg += ", ";
                    }

                    msg += QString::number( mTmcm1637Vec.at( i ).getNodeId() );
                    haveFirst = true;
                }
            }

            QMessageBox::critical( this, "TMCM-BB4 CANopen", msg, QMessageBox::Ok );
        }

        //***********************************************************
        // set state to SWITCHED_ON
        //***********************************************************
        troubleFound = false;

        for( size_t i = 0; i < mTmcm1637Vec.size(); i++ )
        {
            if( mTmcmIdentityVec.at( i ) && mTmcmStateVec.at( i ) )
            {
                Tmcm1637CanOpen::MotionControllerState state = Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_UNKNOWN;
                bool status = mTmcm1637Vec.at( i ).getState( state );

                if( status )
                {
                    mTmcmOperEnVec.at( i ) = ( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_OPERATION_ENABLED == state );

                    if( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_SWITCHED_ON != state )
                    {
                        status = mTmcm1637Vec.at( i ).setState( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_SWITCHED_ON );

                        if( !status )
                        {
                            status = mTmcm1637Vec.at( i ).setState( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_SWITCHED_ON );
                        }

                        mTmcmStateVec.at( i ) = status;

                        if( status )
                        {
                            mTmcmOperEnVec.at( i ) = false;
                        }
                        else
                        {
                            troubleFound = true;
                        }

                        switch( i )
                        {
                            case 0:
                                mMainUi->Operation_0_Button->setText( ENABLE_STR );
                                break;

                            case 1:
                                mMainUi->Operation_1_Button->setText( ENABLE_STR );
                                break;

                            case 2:
                                mMainUi->Operation_2_Button->setText( ENABLE_STR );
                                break;

                            case 3:
                                mMainUi->Operation_3_Button->setText( ENABLE_STR );
                                break;

                            default:
                                break;
                        }
                    }
                }
                else
                {
                    mTmcmStateVec.at( i ) = false;
                    troubleFound = true;
                }                    
            }
        }

        if( troubleFound )
        {
            msg = "Could not set state to SWITCHED_ON for TMCM-1637 modules\nwith node ID: ";
            bool haveFirst = false;

            for( size_t i = 0; i < mTmcm1637Vec.size(); i++ )
            {
                if( !mTmcmStateVec.at( i ) )
                {
                    if( haveFirst )
                    {
                        msg += ", ";
                    }

                    msg += QString::number( mTmcm1637Vec.at( i ).getNodeId() );
                    haveFirst = true;
                }
            }

            QMessageBox::critical( this, "TMCM-BB4 CANopen", msg, QMessageBox::Ok );
        }

        //***********************************************************
        // update UI controls
        //***********************************************************
        int32_t velocityRpm = 0;
        int32_t position = 0;

        if( mTmcm1637Vec.at( 0 ).areMotorParametersSet() )
        {
            mMainUi->Tmcm1637_0_StatusFrame->setStyleSheet( "image: url(:/images/ok.png)" );

            mMainUi->SetVelocity_0_Spin->setEnabled( true );

            if( mTmcm1637Vec.at( 0 ).getTargetVelocity( velocityRpm ) )
            {
                mMainUi->SetVelocity_0_Spin->setValue( velocityRpm );
            }

            mMainUi->SetPosition_0_Spin->setEnabled( true );

            if( mTmcm1637Vec.at( 0 ).getCurrentPosition( position ) )
            {
                mMainUi->SetPosition_0_Spin->setValue( position );
            }

            mMainUi->OperatingMode_0_ComboBox->setEnabled( true );
            mMainUi->Operation_0_Button->setEnabled( mTmcmStateVec.at( 0 ) );
            mMainUi->HomeUpdate_0_Button->setEnabled( mTmcmStateVec.at( 0 ) );
        }
        else
        {
            mMainUi->Tmcm1637_0_StatusFrame->setStyleSheet( "image: url(:/images/fail.png)" );            

            mMainUi->SetVelocity_0_Spin->setEnabled( false );
            mMainUi->SetVelocity_0_Spin->setValue( 0 );

            mMainUi->SetPosition_0_Spin->setEnabled( false );
            mMainUi->SetPosition_0_Spin->setValue( 0 );

            mMainUi->OperatingMode_0_ComboBox->setEnabled( false );            
            mMainUi->Operation_0_Button->setEnabled( false );
            mMainUi->HomeUpdate_0_Button->setEnabled( false );
        }

        if( mTmcm1637Vec.at( 1 ).areMotorParametersSet() )
        {
            mMainUi->Tmcm1637_1_StatusFrame->setStyleSheet( "image: url(:/images/ok.png)" );

            mMainUi->SetVelocity_1_Spin->setEnabled( true );

            if( mTmcm1637Vec.at( 1 ).getTargetVelocity( velocityRpm ) )
            {
                mMainUi->SetVelocity_1_Spin->setValue( velocityRpm );
            }

            mMainUi->SetPosition_1_Spin->setEnabled( true );

            if( mTmcm1637Vec.at( 1 ).getCurrentPosition( position ) )
            {
                mMainUi->SetPosition_1_Spin->setValue( position );
            }

            mMainUi->OperatingMode_1_ComboBox->setEnabled( true );
            mMainUi->Operation_1_Button->setEnabled( mTmcmStateVec.at( 1 ) );
            mMainUi->HomeUpdate_1_Button->setEnabled( mTmcmStateVec.at( 1 ) );
        }
        else
        {
            mMainUi->Tmcm1637_1_StatusFrame->setStyleSheet( "image: url(:/images/fail.png)" );

            mMainUi->SetVelocity_1_Spin->setEnabled( false );
            mMainUi->SetVelocity_1_Spin->setValue( 0 );

            mMainUi->SetPosition_1_Spin->setEnabled( false );
            mMainUi->SetPosition_1_Spin->setValue( 0 );

            mMainUi->OperatingMode_1_ComboBox->setEnabled( false );
            mMainUi->Operation_1_Button->setEnabled( false );
            mMainUi->HomeUpdate_1_Button->setEnabled( false );
        }

        if( mTmcm1637Vec.at( 2 ).areMotorParametersSet() )
        {
            mMainUi->Tmcm1637_2_StatusFrame->setStyleSheet( "image: url(:/images/ok.png)" );

            mMainUi->SetVelocity_2_Spin->setEnabled( true );

            if( mTmcm1637Vec.at( 2 ).getTargetVelocity( velocityRpm ) )
            {
                mMainUi->SetVelocity_2_Spin->setValue( velocityRpm );
            }

            mMainUi->SetPosition_2_Spin->setEnabled( true );

            if( mTmcm1637Vec.at( 2 ).getCurrentPosition( position ) )
            {
                mMainUi->SetPosition_2_Spin->setValue( position );
            }

            mMainUi->OperatingMode_2_ComboBox->setEnabled( true );
            mMainUi->Operation_2_Button->setEnabled( mTmcmStateVec.at( 2 ) );
            mMainUi->HomeUpdate_2_Button->setEnabled( mTmcmStateVec.at( 2 ) );
        }
        else
        {
            mMainUi->Tmcm1637_2_StatusFrame->setStyleSheet( "image: url(:/images/fail.png)" );

            mMainUi->SetVelocity_2_Spin->setEnabled( false );
            mMainUi->SetVelocity_2_Spin->setValue( 0 );

            mMainUi->SetPosition_2_Spin->setEnabled( false );
            mMainUi->SetPosition_2_Spin->setValue( 0 );

            mMainUi->OperatingMode_2_ComboBox->setEnabled( false );
            mMainUi->Operation_2_Button->setEnabled( false );
            mMainUi->HomeUpdate_2_Button->setEnabled( false );
        }

        if( mTmcm1637Vec.at( 3 ).areMotorParametersSet() )
        {
            mMainUi->Tmcm1637_3_StatusFrame->setStyleSheet( "image: url(:/images/ok.png)" );

            mMainUi->SetVelocity_3_Spin->setEnabled( true );

            if( mTmcm1637Vec.at( 3 ).getTargetVelocity( velocityRpm ) )
            {
                mMainUi->SetVelocity_3_Spin->setValue( velocityRpm );
            }

            mMainUi->SetPosition_3_Spin->setEnabled( true );

            if( mTmcm1637Vec.at( 3 ).getCurrentPosition( position ) )
            {
                mMainUi->SetPosition_3_Spin->setValue( position );
            }

            mMainUi->OperatingMode_3_ComboBox->setEnabled( true );
            mMainUi->Operation_3_Button->setEnabled( mTmcmStateVec.at( 3 ) );
            mMainUi->HomeUpdate_3_Button->setEnabled( mTmcmStateVec.at( 3 ) );
        }
        else
        {
            mMainUi->Tmcm1637_3_StatusFrame->setStyleSheet( "image: url(:/images/fail.png)" );

            mMainUi->SetVelocity_3_Spin->setEnabled( false );
            mMainUi->SetVelocity_3_Spin->setValue( 0 );

            mMainUi->SetPosition_3_Spin->setEnabled( false );
            mMainUi->SetPosition_3_Spin->setValue( 0 );

            mMainUi->OperatingMode_3_ComboBox->setEnabled( false );
            mMainUi->Operation_3_Button->setEnabled( false );
            mMainUi->HomeUpdate_3_Button->setEnabled( false );
        }

        //***********************************************************
        // read operating modes
        //***********************************************************
        for( size_t i = 0; i < mTmcm1637Vec.size(); i++ )
        {
            if( mTmcm1637Vec.at( i ).areMotorParametersSet() )
            {
                Tmcm1637CanOpen::OperatingMode operMode;
                bool haveOperatingMode = mTmcm1637Vec.at( i ).getOperatingMode( operMode );

                if( haveOperatingMode )
                {
                    int index = 0;

                    switch( i )
                    {
                        case 0:
                            index = mMainUi->OperatingMode_0_ComboBox->findData( operMode );
                            mMainUi->OperatingMode_0_ComboBox->setCurrentIndex( index );
                            updateControlsForOperatingMode0( operMode );
                            break;

                        case 1:
                            index = mMainUi->OperatingMode_1_ComboBox->findData( operMode );
                            mMainUi->OperatingMode_1_ComboBox->setCurrentIndex( index );
                            updateControlsForOperatingMode1( operMode );
                            break;

                        case 2:
                            index = mMainUi->OperatingMode_2_ComboBox->findData( operMode );
                            mMainUi->OperatingMode_2_ComboBox->setCurrentIndex( index );
                            updateControlsForOperatingMode2( operMode );
                            break;

                        case 3:
                            index = mMainUi->OperatingMode_3_ComboBox->findData( operMode );
                            mMainUi->OperatingMode_3_ComboBox->setCurrentIndex( index );
                            updateControlsForOperatingMode3( operMode );
                            break;

                        default:
                            break;
                    }
                }
            }
        }

        //***********************************************************
        // start timers
        //***********************************************************
        mMotorsPollingTimer->start( 1000 );
    }
}


//!************************************************************************
//! Convert and format a number in hex string so that it contains a minimum
//! number of hex digits
//! e.g. formatHexString( 10, 0xFFF ) -> "0x00A"
//!
//! @returns: The QString containing the number in hex
//!************************************************************************
QString TmcmBb4::formatHexString
    (
    const uint32_t  aValue,     //!< value to convert
    const uint32_t  aMaxValue   //!< maximum expected value
    ) const
{
    QString str = "0x";
    QString valueHexStr = QString::number( aValue, 16 );
    QString maxValueHexStr = QString::number( aMaxValue, 16 );
    int diffZeros = maxValueHexStr.size() - valueHexStr.size();

    for( int i = 0; i < diffZeros; i++ )
    {
        str += "0";
    }

    str += valueHexStr;
    return str;
}


//!************************************************************************
//! Handle for CAN connect event
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void TmcmBb4::handleCanConnect()
{
    if( mCanOpen )
    {
        mMainUi->CanConnectionLabel->setText( "Connecting to CAN bus.." );
        mCanOpen->initCanBus();
        QTimer::singleShot( 50, this, &TmcmBb4::checkMotionControllers );
    }
}


//!************************************************************************
//! Handle for CAN disconnect event
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void TmcmBb4::handleCanDisconnect()
{
    if( mCanOpen )
    {
        if( mCanOpen->getCanDevice() )
        {
            if( mCanOpen->isConnected() )
            {
                for( size_t i = 0; i < mTmcm1637Vec.size(); i++ )
                {
                    if( mTmcmIdentityVec.at( i ) )
                    {
                        mTmcm1637Vec.at( i ).setState( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_SWITCH_ON_DISABLED );
                    }
                }

                mMainUi->CanConnectionLabel->setText( "Disconnecting from CAN bus.." );
                mCanOpen->disconnect();
            }
        }
    }
}


//!************************************************************************
//! Handle for CAN reset
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void TmcmBb4::handleCanReset()
{
    if( mCanOpen )
    {
        mCanOpen->reset();
    }
}


//!************************************************************************
//! Handle for exit event
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void TmcmBb4::handleExit()
{
    handleCanDisconnect();
    QApplication::quit();
}


//!************************************************************************
//! Handle for updating home position 0 (1st controller board)
//!
//! @returns: nothing
//!************************************************************************
void TmcmBb4::handleHomeUpdate0()
{
    if( mCanOpen )
    {
        if( mCanOpen->getCanDevice() )
        {
            int32_t position = 0;

            if( mTmcm1637Vec.at( 0 ).getCurrentPosition( position ) )
            {
                mTmcm1637Vec.at( 0 ).setHomePosition( position );
            }
            else
            {
                qDebug() << "Could not get current position - 1";
            }
        }
    }
}


//!************************************************************************
//! Handle for updating home position 1 (2nd controller board)
//!
//! @returns: nothing
//!************************************************************************
void TmcmBb4::handleHomeUpdate1()
{
    if( mCanOpen )
    {
        if( mCanOpen->getCanDevice() )
        {
            int32_t position = 0;

            if( mTmcm1637Vec.at( 1 ).getCurrentPosition( position ) )
            {
                mTmcm1637Vec.at( 1 ).setHomePosition( position );
            }
            else
            {
                qDebug() << "Could not get current position - 2";
            }
        }
    }
}


//!************************************************************************
//! Handle for updating home position 2 (3rd controller board)
//!
//! @returns: nothing
//!************************************************************************
void TmcmBb4::handleHomeUpdate2()
{
    if( mCanOpen )
    {
        if( mCanOpen->getCanDevice() )
        {
            int32_t position = 0;

            if( mTmcm1637Vec.at( 2 ).getCurrentPosition( position ) )
            {
                mTmcm1637Vec.at( 2 ).setHomePosition( position );
            }
            else
            {
                qDebug() << "Could not get current position - 3";
            }
        }
    }
}


//!************************************************************************
//! Handle for updating home position 3 (4th controller board)
//!
//! @returns: nothing
//!************************************************************************
void TmcmBb4::handleHomeUpdate3()
{
    if( mCanOpen )
    {
        if( mCanOpen->getCanDevice() )
        {
            int32_t position = 0;

            if( mTmcm1637Vec.at( 3 ).getCurrentPosition( position ) )
            {
                mTmcm1637Vec.at( 3 ).setHomePosition( position );
            }
            else
            {
                qDebug() << "Could not get current position - 4";
            }
        }
    }
}


//!************************************************************************
//! Handle for changing the operating mode 0 (1st controller board)
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void TmcmBb4::handleOperatingMode0
    (
    int aIndex  //!< index
    )
{
    if( mCanOpen )
    {
        if( mCanOpen->getCanDevice() )
        {
            Tmcm1637CanOpen::OperatingMode operMode = Tmcm1637CanOpen::OPERATING_MODE_NONE;

            switch( aIndex )
            {
                case 0:
                    operMode = Tmcm1637CanOpen::OPERATING_MODE_NONE;
                    break;

                case 1:
                    operMode = Tmcm1637CanOpen::OPERATING_MODE_PROFILE_POSITION;
                    break;

                case 2:
                    operMode = Tmcm1637CanOpen::OPERATING_MODE_PROFILE_VELOCITY;
                    break;

                case 3:
                    operMode = Tmcm1637CanOpen::OPERATING_MODE_HOMING;
                    break;

                default:
                    break;
            }

            if( Tmcm1637CanOpen::OPERATING_MODE_HOMING == operMode )
            {
                if( !mTmcm1637Vec.at( 0 ).setOperatingMode( Tmcm1637CanOpen::OPERATING_MODE_PROFILE_POSITION ) )
                {
                    qDebug() << "Could not set operating mode - 1";
                }

                if( !mTmcm1637Vec.at( 0 ).goToHomePosition() )
                {
                    qDebug() << "Could not go to home position - 1";
                }
            }
            else
            {
                if( !mTmcm1637Vec.at( 0 ).setOperatingMode( operMode ) )
                {
                    qDebug() << "Could not set operating mode - 1";
                }
            }

            updateControlsForOperatingMode0( operMode );
        }
    }
}


//!************************************************************************
//! Handle for changing the operating mode 1 (2nd controller board)
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void TmcmBb4::handleOperatingMode1
    (
    int aIndex  //!< index
    )
{
    if( mCanOpen )
    {
        if( mCanOpen->getCanDevice() )
        {
            Tmcm1637CanOpen::OperatingMode operMode = Tmcm1637CanOpen::OPERATING_MODE_NONE;

            switch( aIndex )
            {
                case 0:
                    operMode = Tmcm1637CanOpen::OPERATING_MODE_NONE;
                    break;

                case 1:
                    operMode = Tmcm1637CanOpen::OPERATING_MODE_PROFILE_POSITION;
                    break;

                case 2:
                    operMode = Tmcm1637CanOpen::OPERATING_MODE_PROFILE_VELOCITY;
                    break;

                case 3:
                    operMode = Tmcm1637CanOpen::OPERATING_MODE_HOMING;
                    break;

                default:
                    break;
            }

            if( Tmcm1637CanOpen::OPERATING_MODE_HOMING == operMode )
            {
                if( !mTmcm1637Vec.at( 1 ).setOperatingMode( Tmcm1637CanOpen::OPERATING_MODE_PROFILE_POSITION ) )
                {
                    qDebug() << "Could not set operating mode - 2";
                }

                if( !mTmcm1637Vec.at( 1 ).goToHomePosition() )
                {
                    qDebug() << "Could not go to home position - 2";
                }
            }
            else
            {
                if( !mTmcm1637Vec.at( 1 ).setOperatingMode( operMode ) )
                {
                    qDebug() << "Could not set operating mode - 2";
                }
            }

            updateControlsForOperatingMode1( operMode );
        }
    }
}


//!************************************************************************
//! Handle for changing the operating mode 2 (3rd controller board)
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void TmcmBb4::handleOperatingMode2
    (
    int aIndex  //!< index
    )
{
    if( mCanOpen )
    {
        if( mCanOpen->getCanDevice() )
        {
            Tmcm1637CanOpen::OperatingMode operMode = Tmcm1637CanOpen::OPERATING_MODE_NONE;

            switch( aIndex )
            {
                case 0:
                    operMode = Tmcm1637CanOpen::OPERATING_MODE_NONE;
                    break;

                case 1:
                    operMode = Tmcm1637CanOpen::OPERATING_MODE_PROFILE_POSITION;
                    break;

                case 2:
                    operMode = Tmcm1637CanOpen::OPERATING_MODE_PROFILE_VELOCITY;
                    break;

                case 3:
                    operMode = Tmcm1637CanOpen::OPERATING_MODE_HOMING;
                    break;

                default:
                    break;
            }

            if( Tmcm1637CanOpen::OPERATING_MODE_HOMING == operMode )
            {                
                if( !mTmcm1637Vec.at( 2 ).setOperatingMode( Tmcm1637CanOpen::OPERATING_MODE_PROFILE_POSITION ) )
                {
                    qDebug() << "Could not set operating mode - 3";
                }

                if( !mTmcm1637Vec.at( 2 ).goToHomePosition() )
                {
                    qDebug() << "Could not go to home position - 3";
                }
            }
            else
            {
                if( !mTmcm1637Vec.at( 2 ).setOperatingMode( operMode ) )
                {
                    qDebug() << "Could not set operating mode - 3";
                }
            }

            updateControlsForOperatingMode2( operMode );
        }
    }
}


//!************************************************************************
//! Handle for changing the operating mode 3 (4th controller board)
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void TmcmBb4::handleOperatingMode3
    (
    int aIndex  //!< index
    )
{
    if( mCanOpen )
    {
        if( mCanOpen->getCanDevice() )
        {
            Tmcm1637CanOpen::OperatingMode operMode = Tmcm1637CanOpen::OPERATING_MODE_NONE;

            switch( aIndex )
            {
                case 0:
                    operMode = Tmcm1637CanOpen::OPERATING_MODE_NONE;
                    break;

                case 1:
                    operMode = Tmcm1637CanOpen::OPERATING_MODE_PROFILE_POSITION;
                    break;

                case 2:
                    operMode = Tmcm1637CanOpen::OPERATING_MODE_PROFILE_VELOCITY;
                    break;

                case 3:
                    operMode = Tmcm1637CanOpen::OPERATING_MODE_HOMING;
                    break;

                default:
                    break;
            }

            if( Tmcm1637CanOpen::OPERATING_MODE_HOMING == operMode )
            {
                if( !mTmcm1637Vec.at( 3 ).setOperatingMode( Tmcm1637CanOpen::OPERATING_MODE_PROFILE_POSITION ) )
                {
                    qDebug() << "Could not set operating mode - 4";
                }

                if( !mTmcm1637Vec.at( 3 ).goToHomePosition() )
                {
                    qDebug() << "Could not go to home position - 4";
                }
            }
            else
            {
                if( !mTmcm1637Vec.at( 3 ).setOperatingMode( operMode ) )
                {
                    qDebug() << "Could not set operating mode - 4";
                }
            }

            updateControlsForOperatingMode3( operMode );
        }
    }
}


//!************************************************************************
//! Handle for enabling/disabling the operation 0  (1st controller board)
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void TmcmBb4::handleOperationEnable0()
{
    if( mCanOpen )
    {
        if( mCanOpen->getCanDevice() )
        {
            bool isOperationEnabled = mTmcmOperEnVec.at( 0 );

            Tmcm1637CanOpen::MotionControllerState state = Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_UNKNOWN;
            bool status = mTmcm1637Vec.at( 0 ).getState( state );

            if( status )
            {
                isOperationEnabled = ( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_OPERATION_ENABLED == state );
            }

            if( !isOperationEnabled )
            {
                status = mTmcm1637Vec.at( 0 ).setState( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_OPERATION_ENABLED );

                if( !status )
                {
                    status = mTmcm1637Vec.at( 0 ).setState( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_OPERATION_ENABLED );
                }

                if( status )
                {
                    mTmcmOperEnVec.at( 0 ) = true;
                    mMainUi->Operation_0_Button->setText( DISABLE_STR );
                }
            }
            else
            {
                status = mTmcm1637Vec.at( 0 ).setState( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_SWITCHED_ON );

                if( !status )
                {
                    status = mTmcm1637Vec.at( 0 ).setState( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_SWITCHED_ON );
                }

                if( status )
                {
                    mTmcmOperEnVec.at( 0 ) = false;
                    mMainUi->Operation_0_Button->setText( ENABLE_STR );
                }
            }
        }
    }
}


//!************************************************************************
//! Handle for enabling/disabling the operation 1  (2nd controller board)
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void TmcmBb4::handleOperationEnable1()
{
    if( mCanOpen )
    {
        if( mCanOpen->getCanDevice() )
        {                        
            bool isOperationEnabled = mTmcmOperEnVec.at( 1 );

            Tmcm1637CanOpen::MotionControllerState state = Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_UNKNOWN;
            bool status = mTmcm1637Vec.at( 1 ).getState( state );

            if( status )
            {
                isOperationEnabled = ( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_OPERATION_ENABLED == state );
            }

            if( !isOperationEnabled )
            {
                status = mTmcm1637Vec.at( 1 ).setState( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_OPERATION_ENABLED );

                if( !status )
                {
                    status = mTmcm1637Vec.at( 1 ).setState( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_OPERATION_ENABLED );
                }

                if( status )
                {
                    mTmcmOperEnVec.at( 1 ) = true;
                    mMainUi->Operation_1_Button->setText( DISABLE_STR );
                }
            }
            else
            {
                status = mTmcm1637Vec.at( 1 ).setState( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_SWITCHED_ON );

                if( !status )
                {
                    status = mTmcm1637Vec.at( 1 ).setState( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_SWITCHED_ON );
                }

                if( status )
                {
                    mTmcmOperEnVec.at( 1 ) = false;
                    mMainUi->Operation_1_Button->setText( ENABLE_STR );
                }
            }
        }
    }
}


//!************************************************************************
//! Handle for enabling/disabling the operation 2 (3rd controller board)
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void TmcmBb4::handleOperationEnable2()
{
    if( mCanOpen )
    {
        if( mCanOpen->getCanDevice() )
        {
            bool isOperationEnabled = mTmcmOperEnVec.at( 2 );

            Tmcm1637CanOpen::MotionControllerState state = Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_UNKNOWN;
            bool status = mTmcm1637Vec.at( 2 ).getState( state );

            if( status )
            {
                isOperationEnabled = ( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_OPERATION_ENABLED == state );
            }

            if( !isOperationEnabled )
            {
                status = mTmcm1637Vec.at( 2 ).setState( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_OPERATION_ENABLED );

                if( !status )
                {
                    status = mTmcm1637Vec.at( 2 ).setState( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_OPERATION_ENABLED );
                }

                if( status )
                {
                    mTmcmOperEnVec.at( 2 ) = true;
                    mMainUi->Operation_2_Button->setText( DISABLE_STR );
                }
            }
            else
            {
                status = mTmcm1637Vec.at( 2 ).setState( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_SWITCHED_ON );

                if( !status )
                {
                    status = mTmcm1637Vec.at( 2 ).setState( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_SWITCHED_ON );
                }

                if( status )
                {
                    mTmcmOperEnVec.at( 2 ) = false;
                    mMainUi->Operation_2_Button->setText( ENABLE_STR );
                }
            }
        }
    }
}


//!************************************************************************
//! Handle for enabling/disabling the operation 3 (4th controller board)
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void TmcmBb4::handleOperationEnable3()
{
    if( mCanOpen )
    {
        if( mCanOpen->getCanDevice() )
        {
            bool isOperationEnabled = mTmcmOperEnVec.at( 3 );

            Tmcm1637CanOpen::MotionControllerState state = Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_UNKNOWN;
            bool status = mTmcm1637Vec.at( 3 ).getState( state );

            if( status )
            {
                isOperationEnabled = ( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_OPERATION_ENABLED == state );
            }

            if( !isOperationEnabled )
            {
                status = mTmcm1637Vec.at( 3 ).setState( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_OPERATION_ENABLED );

                if( !status )
                {
                    status = mTmcm1637Vec.at( 3 ).setState( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_OPERATION_ENABLED );
                }

                if( status )
                {
                    mTmcmOperEnVec.at( 3 ) = true;
                    mMainUi->Operation_3_Button->setText( DISABLE_STR );
                }
            }
            else
            {
                status = mTmcm1637Vec.at( 3 ).setState( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_SWITCHED_ON );

                if( !status )
                {
                    status = mTmcm1637Vec.at( 3 ).setState( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_SWITCHED_ON );
                }

                if( status )
                {
                    mTmcmOperEnVec.at( 3 ) = false;
                    mMainUi->Operation_3_Button->setText( ENABLE_STR );
                }
            }
        }
    }
}


//!************************************************************************
//! Handle for changing the target position 0 (1st controller board)
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void TmcmBb4::handlePosition0
    (
    int aValue  //!< value
    )
{
    if( mCanOpen )
    {
        if( mCanOpen->getCanDevice() )
        {           
            Tmcm1637CanOpen::MotionControllerState controllerState = Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_UNKNOWN;
            bool status = mTmcm1637Vec.at( 0 ).getState( controllerState );

            if( status )
            {
                status = ( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_OPERATION_ENABLED == controllerState );
            }

            int32_t encodedTargetPosition = mTmcm1637Vec.at( 0 ).getEncodedTargetPosition( aValue );
            mMainUi->SetPosition_0_Spin->setValue( encodedTargetPosition );

            if( status )
            {
                status = mTmcm1637Vec.at( 0 ).setTargetPosition( encodedTargetPosition );
            }

            uint16_t controlWord = 0;

            if( status )
            {
                status = mTmcm1637Vec.at( 0 ).getControlWord( controlWord );
            }

            if( status )
            {
                controlWord |= Tmcm1637CanOpen::CONTROL_WORD_NEW_SET_POINT;
                status = mTmcm1637Vec.at( 0 ).setControlWord( controlWord );
            }

            if( status )
            {
                status = mTmcm1637Vec.at( 0 ).setState( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_OPERATION_ENABLED );
            }

            if( !status )
            {
                if( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_OPERATION_ENABLED == controllerState )
                {
                    qDebug() << "Could not set target position - 1";
                }
            }
        }
    }
}


//!************************************************************************
//! Handle for changing the target position 1 (2nd controller board)
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void TmcmBb4::handlePosition1
    (
    int aValue  //!< value
    )
{
    if( mCanOpen )
    {
        if( mCanOpen->getCanDevice() )
        {
            Tmcm1637CanOpen::MotionControllerState controllerState = Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_UNKNOWN;
            bool status = mTmcm1637Vec.at( 1 ).getState( controllerState );

            if( status )
            {
                status = ( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_OPERATION_ENABLED == controllerState );
            }

            int32_t encodedTargetPosition = mTmcm1637Vec.at( 1 ).getEncodedTargetPosition( aValue );
            mMainUi->SetPosition_1_Spin->setValue( encodedTargetPosition );

            if( status )
            {
                status = mTmcm1637Vec.at( 1 ).setTargetPosition( encodedTargetPosition );
            }

            uint16_t controlWord = 0;

            if( status )
            {
                status = mTmcm1637Vec.at( 1 ).getControlWord( controlWord );
            }

            if( status )
            {
                controlWord |= Tmcm1637CanOpen::CONTROL_WORD_NEW_SET_POINT;
                status = mTmcm1637Vec.at( 1 ).setControlWord( controlWord );
            }

            if( status )
            {
                status = mTmcm1637Vec.at( 1 ).setState( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_OPERATION_ENABLED );
            }

            if( !status )
            {
                if( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_OPERATION_ENABLED == controllerState )
                {
                    qDebug() << "Could not set target position - 2";
                }
            }
        }
    }
}


//!************************************************************************
//! Handle for changing the target position 2 (3rd controller board)
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void TmcmBb4::handlePosition2
    (
    int aValue  //!< value
    )
{
    if( mCanOpen )
    {
        if( mCanOpen->getCanDevice() )
        {
            Tmcm1637CanOpen::MotionControllerState controllerState = Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_UNKNOWN;
            bool status = mTmcm1637Vec.at( 2 ).getState( controllerState );

            if( status )
            {
                status = ( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_OPERATION_ENABLED == controllerState );
            }

            int32_t encodedTargetPosition = mTmcm1637Vec.at( 2 ).getEncodedTargetPosition( aValue );
            mMainUi->SetPosition_2_Spin->setValue( encodedTargetPosition );

            if( status )
            {
                status = mTmcm1637Vec.at( 2 ).setTargetPosition( encodedTargetPosition );
            }

            uint16_t controlWord = 0;

            if( status )
            {
                status = mTmcm1637Vec.at( 2 ).getControlWord( controlWord );
            }

            if( status )
            {
                controlWord |= Tmcm1637CanOpen::CONTROL_WORD_NEW_SET_POINT;
                status = mTmcm1637Vec.at( 2 ).setControlWord( controlWord );
            }

            if( status )
            {
                status = mTmcm1637Vec.at( 2 ).setState( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_OPERATION_ENABLED );
            }

            if( !status )
            {
                if( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_OPERATION_ENABLED == controllerState )
                {
                    qDebug() << "Could not set target position - 3";
                }
            }
        }
    }
}


//!************************************************************************
//! Handle for changing the target position 3 (4th controller board)
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void TmcmBb4::handlePosition3
    (
    int aValue  //!< value
    )
{
    if( mCanOpen )
    {
        if( mCanOpen->getCanDevice() )
        {
            Tmcm1637CanOpen::MotionControllerState controllerState = Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_UNKNOWN;
            bool status = mTmcm1637Vec.at( 3 ).getState( controllerState );

            if( status )
            {
                status = ( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_OPERATION_ENABLED == controllerState );
            }

            int32_t encodedTargetPosition = mTmcm1637Vec.at( 3 ).getEncodedTargetPosition( aValue );
            mMainUi->SetPosition_3_Spin->setValue( encodedTargetPosition );

            if( status )
            {
                status = mTmcm1637Vec.at( 3 ).setTargetPosition( encodedTargetPosition );
            }

            uint16_t controlWord = 0;

            if( status )
            {
                status = mTmcm1637Vec.at( 3 ).getControlWord( controlWord );
            }

            if( status )
            {
                controlWord |= Tmcm1637CanOpen::CONTROL_WORD_NEW_SET_POINT;
                status = mTmcm1637Vec.at( 3 ).setControlWord( controlWord );
            }

            if( status )
            {
                status = mTmcm1637Vec.at( 3 ).setState( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_OPERATION_ENABLED );
            }

            if( !status )
            {
                if( Tmcm1637CanOpen::MOTION_CONTROLLER_STATE_OPERATION_ENABLED == controllerState )
                {
                    qDebug() << "Could not set target position - 4";
                }
            }
        }
    }
}


//!************************************************************************
//! Handle for changing the target velocity 0  (1st controller board)
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void TmcmBb4::handleVelocity0
    (
    int aValue  //!< value
    )
{
    if( mCanOpen )
    {
        if( mCanOpen->getCanDevice() )
        {
            if( !mTmcm1637Vec.at( 0 ).setTargetVelocity( static_cast<int32_t>( aValue ) ) )
            {
                qDebug() << "Could not set target velocity - 1";
            }
        }
    }
}


//!************************************************************************
//! Handle for changing the target velocity 1 (2nd controller board)
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void TmcmBb4::handleVelocity1
    (
    int aValue  //!< value
    )
{
    if( mCanOpen )
    {
        if( mCanOpen->getCanDevice() )
        {
            if( !mTmcm1637Vec.at( 1 ).setTargetVelocity( static_cast<int32_t>( aValue ) ) )
            {
                qDebug() << "Could not set target velocity - 2";
            }
        }
    }
}


//!************************************************************************
//! Handle for changing the target velocity 2 (3rd controller board)
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void TmcmBb4::handleVelocity2
    (
    int aValue  //!< value
    )
{
    if( mCanOpen )
    {
        if( mCanOpen->getCanDevice() )
        {
            if( !mTmcm1637Vec.at( 2 ).setTargetVelocity( static_cast<int32_t>( aValue ) ) )
            {
                qDebug() << "Could not set target velocity - 3";
            }
        }
    }
}


//!************************************************************************
//! Handle for changing the target velocity 3 (4th controller board)
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void TmcmBb4::handleVelocity3
    (
    int aValue  //!< value
    )
{
    if( mCanOpen )
    {
        if( mCanOpen->getCanDevice() )
        {
            if( !mTmcm1637Vec.at( 3 ).setTargetVelocity( static_cast<int32_t>( aValue ) ) )
            {
                qDebug() << "Could not set target velocity - 4";
            }
        }
    }
}


//!************************************************************************
//! Poll the information from motion controllers
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void TmcmBb4::pollMotors()
{
    if( mCanOpen )
    {
        if( mCanOpen->getCanDevice() )
        {
            for( size_t i = 0; i < mTmcm1637Vec.size(); i++ )
            {
                if( mTmcm1637Vec.at( i ).areMotorParametersSet() )
                {
                    uint32_t motorStatus = 0;
                    bool haveMotorStatus = mTmcm1637Vec.at( i ).getMotorStatus( motorStatus );
                    QString motorStatusStr = haveMotorStatus ? formatHexString( motorStatus, 0x3FFFF ) : NA_STR;

                    uint16_t controlWord = 0;
                    bool haveControlWord = mTmcm1637Vec.at( i ).getControlWord( controlWord );
                    QString controlWordStr = haveControlWord ? formatHexString( controlWord, 0xFFFF ) : NA_STR;

                    uint16_t statusWord = 0;
                    bool haveStatusWord = mTmcm1637Vec.at( i ).getStatusWord( statusWord );
                    QString statusWordStr = haveStatusWord ? formatHexString( statusWord, 0xFFFF ) : NA_STR;

                    int32_t crtPosition = 0;
                    bool haveCrtPosition = mTmcm1637Vec.at( i ).getCurrentPosition( crtPosition );
                    QString crtPositionStr = haveCrtPosition ? QString::number( static_cast<int32_t>( crtPosition ) ) : NA_STR;
                    QString crtAngleStr = haveCrtPosition ? QString::number( mTmcm1637Vec.at( i ).convertPositionToAngle( crtPosition ), 'f', 0 ) : NA_STR;

                    int32_t crtVelocity = 0;
                    bool haveCrtVelocity = mTmcm1637Vec.at( i ).getCurrentVelocity( crtVelocity );
                    QString crtVelocityStr = haveCrtVelocity ? QString::number( static_cast<int32_t>( crtVelocity ) ) : NA_STR;

                    switch( i )
                    {
                        case 0:
                            mMainUi->Motor_0_MotorStatus->setText( motorStatusStr );
                            mMainUi->Motor_0_ControlWord->setText( controlWordStr );
                            mMainUi->Motor_0_StatusWord->setText( statusWordStr );
                            mMainUi->Motor_0_Angle->setText( crtAngleStr );
                            mMainUi->Motor_0_Position->setText( crtPositionStr );
                            mMainUi->Motor_0_Velocity->setText( crtVelocityStr );
                            break;

                        case 1:
                            mMainUi->Motor_1_MotorStatus->setText( motorStatusStr );
                            mMainUi->Motor_1_ControlWord->setText( controlWordStr );
                            mMainUi->Motor_1_StatusWord->setText( statusWordStr );
                            mMainUi->Motor_1_Angle->setText( crtAngleStr );
                            mMainUi->Motor_1_Position->setText( crtPositionStr );
                            mMainUi->Motor_1_Velocity->setText( crtVelocityStr );
                            break;

                        case 2:
                            mMainUi->Motor_2_MotorStatus->setText( motorStatusStr );
                            mMainUi->Motor_2_ControlWord->setText( controlWordStr );
                            mMainUi->Motor_2_StatusWord->setText( statusWordStr );
                            mMainUi->Motor_2_Angle->setText( crtAngleStr );
                            mMainUi->Motor_2_Position->setText( crtPositionStr );
                            mMainUi->Motor_2_Velocity->setText( crtVelocityStr );
                            break;

                        case 3:
                            mMainUi->Motor_3_MotorStatus->setText( motorStatusStr );
                            mMainUi->Motor_3_ControlWord->setText( controlWordStr );
                            mMainUi->Motor_3_StatusWord->setText( statusWordStr );
                            mMainUi->Motor_3_Angle->setText( crtAngleStr );
                            mMainUi->Motor_3_Position->setText( crtPositionStr );
                            mMainUi->Motor_3_Velocity->setText( crtVelocityStr );
                            break;

                        default:
                            break;
                    }
                }
            }
        }
    }
}


//!************************************************************************
//! Update the controls related to CAN connection
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void TmcmBb4::updateCanConnectStatus
    (
    const bool aStatus      //!< true if connected
    )
{
    mMainUi->CanConnectButton->setEnabled( !aStatus );
    mMainUi->CanDisconnectButton->setEnabled( aStatus );

    if( !aStatus )
    {
        mMotorsPollingTimer->stop();

        mMainUi->Tmcm1637_0_StatusFrame->setStyleSheet( "" );
        mMainUi->Tmcm1637_1_StatusFrame->setStyleSheet( "" );
        mMainUi->Tmcm1637_2_StatusFrame->setStyleSheet( "" );
        mMainUi->Tmcm1637_3_StatusFrame->setStyleSheet( "" );

        mMainUi->Motor_0_MotorStatus->setText( NA_STR );
        mMainUi->Motor_0_ControlWord->setText( NA_STR );
        mMainUi->Motor_0_StatusWord->setText( NA_STR );
        mMainUi->Motor_0_Angle->setText( NA_STR );
        mMainUi->Motor_0_Position->setText( NA_STR );
        mMainUi->Motor_0_Velocity->setText( NA_STR );
        mMainUi->SetPosition_0_Spin->setEnabled( false );
        mMainUi->SetVelocity_0_Spin->setEnabled( false );
        mMainUi->OperatingMode_0_ComboBox->setEnabled( false );
        mMainUi->Operation_0_Button->setEnabled( false );
        mMainUi->Operation_0_Button->setText( ENABLE_STR );
        mMainUi->HomeUpdate_0_Button->setEnabled( false );

        mMainUi->Motor_1_MotorStatus->setText( NA_STR );
        mMainUi->Motor_1_ControlWord->setText( NA_STR );
        mMainUi->Motor_1_StatusWord->setText( NA_STR );
        mMainUi->Motor_1_Angle->setText( NA_STR );
        mMainUi->Motor_1_Position->setText( NA_STR );
        mMainUi->Motor_1_Velocity->setText( NA_STR );
        mMainUi->SetPosition_1_Spin->setEnabled( false );
        mMainUi->SetVelocity_1_Spin->setEnabled( false );
        mMainUi->OperatingMode_1_ComboBox->setEnabled( false );
        mMainUi->Operation_1_Button->setEnabled( false );
        mMainUi->Operation_1_Button->setText( ENABLE_STR );
        mMainUi->HomeUpdate_1_Button->setEnabled( false );

        mMainUi->Motor_2_MotorStatus->setText( NA_STR );
        mMainUi->Motor_2_ControlWord->setText( NA_STR );
        mMainUi->Motor_2_StatusWord->setText( NA_STR );
        mMainUi->Motor_2_Angle->setText( NA_STR );
        mMainUi->Motor_2_Position->setText( NA_STR );
        mMainUi->Motor_2_Velocity->setText( NA_STR );
        mMainUi->SetPosition_2_Spin->setEnabled( false );
        mMainUi->SetVelocity_2_Spin->setEnabled( false );
        mMainUi->OperatingMode_2_ComboBox->setEnabled( false );
        mMainUi->Operation_2_Button->setEnabled( false );
        mMainUi->Operation_2_Button->setText( ENABLE_STR );
        mMainUi->HomeUpdate_2_Button->setEnabled( false );

        mMainUi->Motor_3_MotorStatus->setText( NA_STR );
        mMainUi->Motor_3_ControlWord->setText( NA_STR );
        mMainUi->Motor_3_StatusWord->setText( NA_STR );
        mMainUi->Motor_3_Angle->setText( NA_STR );
        mMainUi->Motor_3_Position->setText( NA_STR );
        mMainUi->Motor_3_Velocity->setText( NA_STR );
        mMainUi->SetPosition_3_Spin->setEnabled( false );
        mMainUi->SetVelocity_3_Spin->setEnabled( false );
        mMainUi->OperatingMode_3_ComboBox->setEnabled( false );
        mMainUi->Operation_3_Button->setEnabled( false );
        mMainUi->Operation_3_Button->setText( ENABLE_STR );
        mMainUi->HomeUpdate_3_Button->setEnabled( false );
    }
}


//!************************************************************************
//! Update the CAN connection string
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void TmcmBb4::updateCanConnectionStr
    (
    const QString aString   //!< string
    )
{
    mMainUi->CanConnectionLabel->setText( aString );
}


//!************************************************************************
//! Update the CAN status string
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void TmcmBb4::updateCanStatusStr
    (
    const QString aString   //!< string
    )
{
    mMainUi->CanStatusLabel->setText( aString );
}


//!************************************************************************
//! Update the UI controls for operating mode 0 (1st controller board)
//!
//! @returns: nothing
//!************************************************************************
void TmcmBb4::updateControlsForOperatingMode0
    (
    const Tmcm1637CanOpen::OperatingMode aOperMode //!< operating mode
    )
{
    switch( aOperMode )
    {
        case Tmcm1637CanOpen::OPERATING_MODE_PROFILE_POSITION:
            mMainUi->SetPosition_0_Spin->setEnabled( true );
            mMainUi->SetVelocity_0_Spin->setEnabled( false );
            break;

        case Tmcm1637CanOpen::OPERATING_MODE_PROFILE_VELOCITY:
            mMainUi->SetPosition_0_Spin->setEnabled( false );
            mMainUi->SetVelocity_0_Spin->setEnabled( true );
            break;

        case Tmcm1637CanOpen::OPERATING_MODE_HOMING:
        case Tmcm1637CanOpen::OPERATING_MODE_NONE:
        default:
            mMainUi->SetPosition_0_Spin->setEnabled( false );
            mMainUi->SetVelocity_0_Spin->setEnabled( false );
            break;
    }
}


//!************************************************************************
//! Update the UI controls for operating mode 1 (2nd controller board)
//!
//! @returns: nothing
//!************************************************************************
void TmcmBb4::updateControlsForOperatingMode1
    (
    const Tmcm1637CanOpen::OperatingMode aOperMode //!< operating mode
    )
{
    switch( aOperMode )
    {
        case Tmcm1637CanOpen::OPERATING_MODE_PROFILE_POSITION:
            mMainUi->SetPosition_1_Spin->setEnabled( true );
            mMainUi->SetVelocity_1_Spin->setEnabled( false );
            break;

        case Tmcm1637CanOpen::OPERATING_MODE_PROFILE_VELOCITY:
            mMainUi->SetPosition_1_Spin->setEnabled( false );
            mMainUi->SetVelocity_1_Spin->setEnabled( true );
            break;

        case Tmcm1637CanOpen::OPERATING_MODE_HOMING:
        case Tmcm1637CanOpen::OPERATING_MODE_NONE:
        default:
            mMainUi->SetPosition_1_Spin->setEnabled( false );
            mMainUi->SetVelocity_1_Spin->setEnabled( false );
            break;
    }
}


//!************************************************************************
//! Update the UI controls for operating mode 2 (3rd controller board)
//!
//! @returns: nothing
//!************************************************************************
void TmcmBb4::updateControlsForOperatingMode2
    (
    const Tmcm1637CanOpen::OperatingMode aOperMode //!< operating mode
    )
{
    switch( aOperMode )
    {
        case Tmcm1637CanOpen::OPERATING_MODE_PROFILE_POSITION:
            mMainUi->SetPosition_2_Spin->setEnabled( true );
            mMainUi->SetVelocity_2_Spin->setEnabled( false );
            break;

        case Tmcm1637CanOpen::OPERATING_MODE_PROFILE_VELOCITY:
            mMainUi->SetPosition_2_Spin->setEnabled( false );
            mMainUi->SetVelocity_2_Spin->setEnabled( true );
            break;

        case Tmcm1637CanOpen::OPERATING_MODE_HOMING:
        case Tmcm1637CanOpen::OPERATING_MODE_NONE:
        default:
            mMainUi->SetPosition_2_Spin->setEnabled( false );
            mMainUi->SetVelocity_2_Spin->setEnabled( false );
            break;
    }
}


//!************************************************************************
//! Update the UI controls for operating mode 3 (4th controller board)
//!
//! @returns: nothing
//!************************************************************************
void TmcmBb4::updateControlsForOperatingMode3
    (
    const Tmcm1637CanOpen::OperatingMode aOperMode //!< operating mode
    )
{
    switch( aOperMode )
    {
        case Tmcm1637CanOpen::OPERATING_MODE_PROFILE_POSITION:
            mMainUi->SetPosition_3_Spin->setEnabled( true );
            mMainUi->SetVelocity_3_Spin->setEnabled( false );
            break;

        case Tmcm1637CanOpen::OPERATING_MODE_PROFILE_VELOCITY:
            mMainUi->SetPosition_3_Spin->setEnabled( false );
            mMainUi->SetVelocity_3_Spin->setEnabled( true );
            break;

        case Tmcm1637CanOpen::OPERATING_MODE_HOMING:
        case Tmcm1637CanOpen::OPERATING_MODE_NONE:
        default:
            mMainUi->SetPosition_3_Spin->setEnabled( false );
            mMainUi->SetVelocity_3_Spin->setEnabled( false );
            break;
    }
}
