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

This file contains the sources for testing CAN bus communication.
*/

#include "CanBusTest.h"
#include "./ui_CanBusTest.h"

#include <cstdlib>
#include <ctime>

#include <QButtonGroup>
#include <QDateTime>
#include <QMessageBox>
#include <QThread>


//!************************************************************************
//! Constructor
//!************************************************************************
CanBusTest::CanBusTest
    (
    QWidget* aParent    //!< parent widget
    )
    : QMainWindow( aParent )
    , mMainUi( new Ui::CanBusTest )
    , mLoopbackActive( false )
    , mMode( MODE_TX )
    , mTestFramesCount( 1000 )
{
    mMainUi->setupUi( this );

    mCanOpen = CanOpen::getInstance();

    // exit
    connect( mMainUi->ExitButton, SIGNAL( clicked() ), this, SLOT( handleExit() ) );

    // CANopen signals
    connect( mCanOpen, &CanOpen::connectionStatusChanged, this, &CanBusTest::updateCanConnectStatus );
    connect( mCanOpen, &CanOpen::connectionStrChanged, this, &CanBusTest::updateCanConnectionStr );
    connect( mCanOpen, &CanOpen::statusStrChanged, this, &CanBusTest::updateCanStatusStr );

    // loopback
    mMainUi->LoopbackCheckbox->setChecked( mLoopbackActive );
    connect( mMainUi->LoopbackCheckbox, SIGNAL( clicked(bool) ), this, SLOT( handleLoopbackChanged(bool) ) );

    // CAN buttons
    connect( mMainUi->CanConnectButton, SIGNAL( clicked() ), this, SLOT( handleCanConnect() ) );
    connect( mMainUi->CanDisconnectButton, SIGNAL( clicked() ), this, SLOT( handleCanDisconnect() ) );
    connect( mMainUi->CanResetButton, SIGNAL( clicked() ), this, SLOT( handleCanReset() ) );

    mMainUi->CanDisconnectButton->setEnabled( false );

    // mode - Tx or Rx
    QButtonGroup* modeBtnGroup = new QButtonGroup( this );
    modeBtnGroup->addButton( mMainUi->TxRadiobutton, MODE_TX );
    modeBtnGroup->addButton( mMainUi->RxRadiobutton, MODE_RX );
    mMainUi->TxRadiobutton->setChecked( MODE_TX == mMode );
    mMainUi->RxRadiobutton->setChecked( MODE_RX == mMode );

    connect( modeBtnGroup, SIGNAL( idClicked(int) ), this, SLOT( updateMode(int) ) );
    mMainUi->ModeGroupbox->setEnabled( false );

    mMainUi->TxGroupbox->setEnabled( MODE_TX == mMode );

    // Tx test actions
    mMainUi->FramesSpinbox->setValue( mTestFramesCount );
    connect( mMainUi->FramesSpinbox, SIGNAL( valueChanged(int) ), this, SLOT( handleFramesCountChanged(int) ) );

    mMainUi->StartButton->setEnabled( false );
    connect( mMainUi->StartButton, SIGNAL( clicked() ), this, SLOT( handleStart() ) );

    // CAN status
    mMainUi->CanConnectionLabel->setText( "" );
    mMainUi->CanStatusLabel->setText( "" );

    // rand() seed
    std::srand( std::time( {} ) );
}


//!************************************************************************
//! Destructor
//!************************************************************************
CanBusTest::~CanBusTest()
{
    delete mMainUi;
}


//!************************************************************************
//! Return a random byte
//!
//! @returns: a random uint8_t
//!************************************************************************
uint8_t CanBusTest::getRandomByte() const
{
    return std::rand() % 128;
}


//!************************************************************************
//! Handle for CAN connect event
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void CanBusTest::handleCanConnect()
{
    if( mCanOpen )
    {
        if( mLoopbackActive )
        {
            QString msg = "Make sure the mode on the other terminal is\n"
                          "set to Tx and connection is active.";
            QMessageBox msgBox;
            msgBox.setText( msg );
            msgBox.exec();
        }

        mMainUi->CanConnectionLabel->setText( "Connecting to CAN bus.." );
        mCanOpen->initCanBus( mLoopbackActive );

        if( mCanOpen->isConnected() )
        {
            mMainUi->ModeGroupbox->setEnabled( !mLoopbackActive );
        }
    }
}


//!************************************************************************
//! Handle for CAN disconnect event
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void CanBusTest::handleCanDisconnect()
{
    if( mCanOpen )
    {
        if( mCanOpen->getCanDevice() )
        {
            mMainUi->CanConnectionLabel->setText( "Disconnecting from CAN bus.." );
            mCanOpen->disconnect();
        }
    }
}


//!************************************************************************
//! Handle for CAN reset
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void CanBusTest::handleCanReset()
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
/* slot */ void CanBusTest::handleExit()
{
    handleCanDisconnect();
    QApplication::quit();
}



//!************************************************************************
//! Handle for changing the number of frames
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void CanBusTest::handleFramesCountChanged
    (
    int aValue  //!< number of frames
    )
{
    mTestFramesCount = aValue;
}


//!************************************************************************
//! Handle for changing the loopback status
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void CanBusTest::handleLoopbackChanged
    (
    bool aStatus    //!< true if checked
    )
{
    mLoopbackActive = aStatus;

    if( mLoopbackActive )
    {
        mMainUi->TxRadiobutton->setChecked( true );
        updateMode( MODE_TX );
    }
}


//!************************************************************************
//! Handle for starting the test
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void CanBusTest::handleStart()
{
    mMainUi->ModeGroupbox->setEnabled( false );
    mMainUi->TxGroupbox->setEnabled( false );

    bool status = false;
    uint16_t frameId = 0;
    uint16_t object = 0;
    uint8_t subindex = 0;
    uint32_t dwToWrite = 0;
    uint32_t failCount = 0;

    if( mCanOpen )
    {
        mMainUi->TestLabel->setText( "Testing, please wait.." );
        QDateTime timeStart = QDateTime::currentDateTime();

        for( size_t i = 0; i < mTestFramesCount; i++ )
        {
            frameId = ( ( getRandomByte() & 7 ) << 8  ) | getRandomByte();
            object = ( getRandomByte() << 8 ) | getRandomByte();
            subindex = getRandomByte();
            dwToWrite = ( getRandomByte() << 24 ) | ( getRandomByte() << 16 ) | ( getRandomByte() << 8 ) | getRandomByte();
            status = mCanOpen->testFrame( frameId, object, subindex, dwToWrite, mLoopbackActive );

            if( !status )
            {
                failCount++;
            }
        }

        QDateTime timeStop = QDateTime::currentDateTime();
        double msDiff = timeStart.msecsTo( timeStop );
        double msPerFrame = msDiff / mTestFramesCount;
        QString msg;

        if( 0 == failCount )
        {
            msg = "Test PASSED";
            msg += ", " + QString::number( msPerFrame ) + " ms/frame";
            mMainUi->TestLabel->setText( msg );
        }
        else
        {
            double failRatio = static_cast<double>( failCount ) / mTestFramesCount;
            msg = QString::number( failCount ) + " frames (" + QString::number( failRatio * 100.0 ) + "%) failed";
            msg += ", " + QString::number( msPerFrame ) + " ms/frame";
            mMainUi->TestLabel->setText( msg );
        }
    }

    mMainUi->TxGroupbox->setEnabled( true );
    mMainUi->ModeGroupbox->setEnabled( !mLoopbackActive );
}


//!************************************************************************
//! Update the controls related to CAN connection
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void CanBusTest::updateCanConnectStatus
    (
    const bool aStatus      //!< true if connected
    )
{
    mMainUi->LoopbackCheckbox->setEnabled( !aStatus );

    mMainUi->CanConnectButton->setEnabled( !aStatus );
    mMainUi->CanDisconnectButton->setEnabled( aStatus );

    mMainUi->ModeGroupbox->setEnabled( aStatus && !mLoopbackActive );

    mMainUi->StartButton->setEnabled( aStatus );
}


//!************************************************************************
//! Update the CAN connection string
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void CanBusTest::updateCanConnectionStr
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
/* slot */ void CanBusTest::updateCanStatusStr
    (
    const QString aString   //!< string
    )
{
    mMainUi->CanStatusLabel->setText( aString );
}


//!************************************************************************
//! Update the Tx/Rx operating mode
//!
//! @returns: nothing
//!************************************************************************
/* slot */ void CanBusTest::updateMode
    (
    int aIndex   //!< operating mode index
    )
{
    mMode = static_cast<Mode>( aIndex );
    mMainUi->TxGroupbox->setEnabled( MODE_TX == mMode );

    if( mCanOpen )
    {
        mCanOpen->connectProcessRxFrames( MODE_RX == mMode );
    }

    mMainUi->TestLabel->setText( MODE_RX == mMode ? "Rx mode" : "" );
}
