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
RobotArmKinematic.cpp

This file contains the sources for the robot arm kinematic.
*/

#include "RobotArmKinematic.h"
#include "./ui_RobotArmKinematic.h"
#include "UrdfHandler.h"
#include "XrdfHandler.h"

#include <cmath>
#include <iostream>

#include <QButtonGroup>
#include <QMessageBox>


//!************************************************************************
//! Constructor
//!************************************************************************
RobotArmKinematic::RobotArmKinematic
    (
    QWidget*    aParent //!< parent widget
    )
    : QMainWindow( aParent )
    , mMainUi( new Ui::RobotArmKinematic )
    , mKinematicType( KINEMATIC_TYPE_DIRECT )
{
    mMainUi->setupUi( this );

    mRobotArmInstance = SerialRobot::getInstance();
    mRobotPartsVec = mRobotArmInstance->getRobotPartsVec();

    mMainUi->TotalLengthValue->setText( QString::number( mRobotArmInstance->getTotalLength(), 'f', 3 ) );

    mMainUi->BaseLengthSpinBox->setValue( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_LINK ).link.geometry.cylinder.length );
    mMainUi->ColumnLengthSpinBox->setValue( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_COLUMN_LINK ).link.geometry.cylinder.length );
    mMainUi->UpperArmLengthSpinBox->setValue( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_UPPER_ARM_LINK ).link.geometry.cylinder.length );
    mMainUi->LowerArmLengthSpinBox->setValue( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_LOWER_ARM_LINK ).link.geometry.cylinder.length );
    mMainUi->EndEffectorLengthSpinBox->setValue( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_END_EFFECTOR_LINK ).link.geometry.cylinder.length );

    mMainUi->BaseRadiusSpinBox->setValue( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_LINK ).link.geometry.cylinder.radius );
    mMainUi->ColumnRadiusSpinBox->setValue( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_COLUMN_LINK ).link.geometry.cylinder.radius );
    mMainUi->UpperArmRadiusSpinBox->setValue( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_UPPER_ARM_LINK ).link.geometry.cylinder.radius );
    mMainUi->LowerArmRadiusSpinBox->setValue( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_LOWER_ARM_LINK ).link.geometry.cylinder.radius );
    mMainUi->EndEffectorRadiusSpinBox->setValue( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_END_EFFECTOR_LINK ).link.geometry.cylinder.radius );

    mMainUi->BaseMassSpinBox->setValue( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_LINK ).link.mass );
    mMainUi->ColumnMassSpinBox->setValue( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_COLUMN_LINK ).link.mass );
    mMainUi->UpperArmMassSpinBox->setValue( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_UPPER_ARM_LINK ).link.mass );
    mMainUi->LowerArmMassSpinBox->setValue( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_LOWER_ARM_LINK ).link.mass );
    mMainUi->EndEffectorMassSpinBox->setValue( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_END_EFFECTOR_LINK ).link.mass );


    connect( mMainUi->BaseLengthSpinBox, SIGNAL( valueChanged(double) ), this, SLOT( handleBaseLengthChanged(double) ) );
    connect( mMainUi->ColumnLengthSpinBox, SIGNAL( valueChanged(double) ), this, SLOT( handleColumnLengthChanged(double) ) );
    connect( mMainUi->UpperArmLengthSpinBox, SIGNAL( valueChanged(double) ), this, SLOT( handleUpperArmLengthChanged(double) ) );
    connect( mMainUi->LowerArmLengthSpinBox, SIGNAL( valueChanged(double) ), this, SLOT( handleLowerArmLengthChanged(double) ) );
    connect( mMainUi->EndEffectorLengthSpinBox, SIGNAL( valueChanged(double) ), this, SLOT( handleEndEffectorLengthChanged(double) ) );

    connect( mMainUi->BaseRadiusSpinBox, SIGNAL( valueChanged(double) ), this, SLOT( handleBaseRadiusChanged(double) ) );
    connect( mMainUi->ColumnRadiusSpinBox, SIGNAL( valueChanged(double) ), this, SLOT( handleColumnRadiusChanged(double) ) );
    connect( mMainUi->UpperArmRadiusSpinBox, SIGNAL( valueChanged(double) ), this, SLOT( handleUpperArmRadiusChanged(double) ) );
    connect( mMainUi->LowerArmRadiusSpinBox, SIGNAL( valueChanged(double) ), this, SLOT( handleLowerArmRadiusChanged(double) ) );
    connect( mMainUi->EndEffectorRadiusSpinBox, SIGNAL( valueChanged(double) ), this, SLOT( handleEndEffectorRadiusChanged(double) ) );

    mMainUi->BaseMassSpinBox->setEnabled( !SerialRobot::USE_FIXED_BASE_MASS );

    if( !SerialRobot::USE_FIXED_BASE_MASS )
    {
        connect( mMainUi->BaseMassSpinBox, SIGNAL( valueChanged(double) ), this, SLOT( handleBaseMassChanged(double) ) );
    }

    connect( mMainUi->ColumnMassSpinBox, SIGNAL( valueChanged(double) ), this, SLOT( handleColumnMassChanged(double) ) );
    connect( mMainUi->UpperArmMassSpinBox, SIGNAL( valueChanged(double) ), this, SLOT( handleUpperArmMassChanged(double) ) );
    connect( mMainUi->LowerArmMassSpinBox, SIGNAL( valueChanged(double) ), this, SLOT( handleLowerArmMassChanged(double) ) );
    connect( mMainUi->EndEffectorMassSpinBox, SIGNAL( valueChanged(double) ), this, SLOT( handleEndEffectorMassChanged(double) ) );


    QButtonGroup* kinematicTypeBtnGroup = new QButtonGroup( this );
    kinematicTypeBtnGroup->addButton( mMainUi->KinematicDirectRadioButton , KINEMATIC_TYPE_DIRECT );
    kinematicTypeBtnGroup->addButton( mMainUi->KinematicInverseRadioButton, KINEMATIC_TYPE_INVERSE );

    connect( kinematicTypeBtnGroup, SIGNAL( idClicked(int) ), this, SLOT( handleKinematicType(int) ) );

    switch( mKinematicType )
    {
        case KINEMATIC_TYPE_DIRECT:
            mMainUi->KinematicDirectRadioButton->setChecked( true );        

            makeReadonly( mMainUi->JointBaseMinSpinBox, false );
            makeReadonly( mMainUi->JointBaseCrtSpinBox, false, true );
            makeReadonly( mMainUi->JointBaseMaxSpinBox, false );

            makeReadonly( mMainUi->JointShoulderMinSpinBox, false );
            makeReadonly( mMainUi->JointShoulderCrtSpinBox, false, true );
            makeReadonly( mMainUi->JointShoulderMaxSpinBox, false );

            makeReadonly( mMainUi->JointElbowMinSpinBox, false );
            makeReadonly( mMainUi->JointElbowCrtSpinBox, false, true );
            makeReadonly( mMainUi->JointElbowMaxSpinBox, false );

            makeReadonly( mMainUi->JointWristMinSpinBox, false );
            makeReadonly( mMainUi->JointWristCrtSpinBox, false, true );
            makeReadonly( mMainUi->JointWristMaxSpinBox, false );

            makeReadonly( mMainUi->EeXSpinBox, true, true );
            makeReadonly( mMainUi->EeYSpinBox, true, true );
            makeReadonly( mMainUi->EeZSpinBox, true, true );

            mMainUi->StatusIkmLabel->setVisible( false );
            break;

        case KINEMATIC_TYPE_INVERSE:
            mMainUi->KinematicInverseRadioButton->setChecked( true );

            makeReadonly( mMainUi->JointBaseMinSpinBox, true );
            makeReadonly( mMainUi->JointBaseCrtSpinBox, true, true );
            makeReadonly( mMainUi->JointBaseMaxSpinBox, true );

            makeReadonly( mMainUi->JointShoulderMinSpinBox, true );
            makeReadonly( mMainUi->JointShoulderCrtSpinBox, true, true );
            makeReadonly( mMainUi->JointShoulderMaxSpinBox, true );

            makeReadonly( mMainUi->JointElbowMinSpinBox, true );
            makeReadonly( mMainUi->JointElbowCrtSpinBox, true, true );
            makeReadonly( mMainUi->JointElbowMaxSpinBox, true );

            makeReadonly( mMainUi->JointWristMinSpinBox, true );
            makeReadonly( mMainUi->JointWristCrtSpinBox, true, true );
            makeReadonly( mMainUi->JointWristMaxSpinBox, true );

            makeReadonly( mMainUi->EeXSpinBox, false, true );
            makeReadonly( mMainUi->EeYSpinBox, false, true );
            makeReadonly( mMainUi->EeZSpinBox, false, true );

            mMainUi->StatusIkmLabel->setVisible( true );
            break;

        default:
            break;
    }

    //*****************************
    // base joint angles
    //*****************************
    mMainUi->JointBaseMinSpinBox->setMinimum( convertRadToDeg( SerialRobot::MIN_BASE_JOINT_ANGLE_RAD ) );
    mMainUi->JointBaseMinSpinBox->setValue( convertRadToDeg( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_JOINT ).joint.limit.lowerAngleRad ) );
    mMainUi->JointBaseMinSpinBox->setMaximum( convertRadToDeg( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_JOINT ).joint.thetaAngleRad ) );

    mMainUi->JointBaseCrtSpinBox->setMinimum( convertRadToDeg( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_JOINT ).joint.limit.lowerAngleRad ) );
    mMainUi->JointBaseCrtSpinBox->setValue( convertRadToDeg( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_JOINT ).joint.thetaAngleRad ) );
    mMainUi->JointBaseCrtSpinBox->setMaximum( convertRadToDeg( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_JOINT ).joint.limit.upperAngleRad ) );

    mMainUi->JointBaseMaxSpinBox->setMinimum( convertRadToDeg( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_JOINT ).joint.thetaAngleRad ) );
    mMainUi->JointBaseMaxSpinBox->setValue( convertRadToDeg( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_JOINT ).joint.limit.upperAngleRad ) );
    mMainUi->JointBaseMaxSpinBox->setMaximum( convertRadToDeg( SerialRobot::MAX_BASE_JOINT_ANGLE_RAD ) );

    //*****************************
    // shoulder joint angles
    //*****************************
    mMainUi->JointShoulderMinSpinBox->setValue( convertRadToDeg( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad ) );
    mMainUi->JointShoulderMinSpinBox->setMaximum( convertRadToDeg( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.thetaAngleRad ) );

    mMainUi->JointShoulderCrtSpinBox->setMinimum( convertRadToDeg( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad ) );
    mMainUi->JointShoulderCrtSpinBox->setValue( convertRadToDeg( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.thetaAngleRad ) );
    mMainUi->JointShoulderCrtSpinBox->setMaximum( convertRadToDeg( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad ) );

    mMainUi->JointShoulderMaxSpinBox->setMinimum( convertRadToDeg( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.thetaAngleRad ) );
    mMainUi->JointShoulderMaxSpinBox->setValue( convertRadToDeg( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad ) );

    //*****************************
    // elbow joint angles
    //*****************************
    mMainUi->JointElbowMinSpinBox->setValue( convertRadToDeg( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad ) );
    mMainUi->JointElbowMinSpinBox->setMaximum( convertRadToDeg( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.thetaAngleRad ) );

    mMainUi->JointElbowCrtSpinBox->setMinimum( convertRadToDeg( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad ) );
    mMainUi->JointElbowCrtSpinBox->setValue( convertRadToDeg( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.thetaAngleRad ) );
    mMainUi->JointElbowCrtSpinBox->setMaximum( convertRadToDeg( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad ) );

    mMainUi->JointElbowMaxSpinBox->setMinimum( convertRadToDeg( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.thetaAngleRad ) );
    mMainUi->JointElbowMaxSpinBox->setValue( convertRadToDeg( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad ) );

    //*****************************
    // wrist joint angles
    //*****************************
    mMainUi->JointWristMinSpinBox->setValue( convertRadToDeg( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.lowerAngleRad ) );
    mMainUi->JointWristMinSpinBox->setMaximum( convertRadToDeg( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.thetaAngleRad ) );

    mMainUi->JointWristCrtSpinBox->setMinimum( convertRadToDeg( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.lowerAngleRad ) );
    mMainUi->JointWristCrtSpinBox->setValue( convertRadToDeg( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.thetaAngleRad ) );
    mMainUi->JointWristCrtSpinBox->setMaximum( convertRadToDeg( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.upperAngleRad ) );

    mMainUi->JointWristMaxSpinBox->setMinimum( convertRadToDeg( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.thetaAngleRad ) );
    mMainUi->JointWristMaxSpinBox->setValue( convertRadToDeg( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.upperAngleRad ) );


    //*****************************
    // Connectors - joint angles
    //*****************************
    connect( mMainUi->JointBaseMinSpinBox, SIGNAL( valueChanged(double) ), this, SLOT( handleJointBaseAngleMinChanged(double) ) );
    connect( mMainUi->JointBaseCrtSpinBox, SIGNAL( valueChanged(double) ), this, SLOT( handleJointBaseAngleCrtChanged(double) ) );
    connect( mMainUi->JointBaseMaxSpinBox, SIGNAL( valueChanged(double) ), this, SLOT( handleJointBaseAngleMaxChanged(double) ) );

    connect( mMainUi->JointShoulderMinSpinBox, SIGNAL( valueChanged(double) ), this, SLOT( handleJointShoulderAngleMinChanged(double) ) );
    connect( mMainUi->JointShoulderCrtSpinBox, SIGNAL( valueChanged(double) ), this, SLOT( handleJointShoulderAngleCrtChanged(double) ) );
    connect( mMainUi->JointShoulderMaxSpinBox, SIGNAL( valueChanged(double) ), this, SLOT( handleJointShoulderAngleMaxChanged(double) ) );

    connect( mMainUi->JointElbowMinSpinBox, SIGNAL( valueChanged(double) ), this, SLOT( handleJointElbowAngleMinChanged(double) ) );
    connect( mMainUi->JointElbowCrtSpinBox, SIGNAL( valueChanged(double) ), this, SLOT( handleJointElbowAngleCrtChanged(double) ) );
    connect( mMainUi->JointElbowMaxSpinBox, SIGNAL( valueChanged(double) ), this, SLOT( handleJointElbowAngleMaxChanged(double) ) );

    connect( mMainUi->JointWristMinSpinBox, SIGNAL( valueChanged(double) ), this, SLOT( handleJointWristAngleMinChanged(double) ) );
    connect( mMainUi->JointWristCrtSpinBox, SIGNAL( valueChanged(double) ), this, SLOT( handleJointWristAngleCrtChanged(double) ) );
    connect( mMainUi->JointWristMaxSpinBox, SIGNAL( valueChanged(double) ), this, SLOT( handleJointWristAngleMaxChanged(double) ) );


    //*****************************
    // End-Effector
    //*****************************
    updateEndEffectorCoordinates();

    connect( mMainUi->EeXSpinBox, SIGNAL( valueChanged(double) ), this, SLOT( handleEeCoordXChanged(double) ) );
    connect( mMainUi->EeYSpinBox, SIGNAL( valueChanged(double) ), this, SLOT( handleEeCoordYChanged(double) ) );
    connect( mMainUi->EeZSpinBox, SIGNAL( valueChanged(double) ), this, SLOT( handleEeCoordZChanged(double) ) );


    //*****************************
    // Robot Descriptor Format generation
    //*****************************
    connect( mMainUi->GenerateUrdfButton, SIGNAL( clicked() ), this, SLOT( handleGenerateUrdf() ) );
    connect( mMainUi->GenerateXrdfButton, SIGNAL( clicked() ), this, SLOT( handleGenerateXrdf() ) );
}


//!************************************************************************
//! Destructor
//!************************************************************************
RobotArmKinematic::~RobotArmKinematic()
{
    delete mMainUi;
}


//!************************************************************************
//! Check if the joint angles are in the current allowed ranges
//!
//! @returns true if all joint angles are in their allowed ranges
//!************************************************************************
bool RobotArmKinematic::checkJointAngles
    (
    double& aTh2,  //!< angle of base joint [rad]
    double& aTh3,  //!< angle of shoulder joint [rad]
    double& aTh4,  //!< angle of elbow joint [rad]
    double& aTh5   //!< angle of wrist joint [rad]
    ) const
{
    bool status = true;

    if( status )
    {
        if( aTh2 >= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_JOINT ).joint.limit.lowerAngleRad - ZERO_RAD_THD
         && aTh2 < mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_JOINT ).joint.limit.lowerAngleRad )
        {
            aTh2 = mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_JOINT ).joint.limit.lowerAngleRad;
        }
        else if( aTh2 < mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_JOINT ).joint.limit.lowerAngleRad - ZERO_RAD_THD )
        {
            status = false;
        }
    }
    if( status )
    {
        if( aTh2 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_JOINT ).joint.limit.upperAngleRad + ZERO_RAD_THD
         && aTh2 > mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_JOINT ).joint.limit.upperAngleRad )
        {
            aTh2 = mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_JOINT ).joint.limit.upperAngleRad;
        }
        else if( aTh2 > mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_JOINT ).joint.limit.upperAngleRad + ZERO_RAD_THD )
        {
            status = false;
        }
    }


    if( status )
    {
        if( aTh3 >= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad - ZERO_RAD_THD
         && aTh3 < mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad )
        {
            aTh3 = mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad;
        }
        else if( aTh3 < mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad - ZERO_RAD_THD )
        {
            status = false;
        }
    }
    if( status )
    {
        if( aTh3 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad + ZERO_RAD_THD
         && aTh3 > mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad )
        {
            aTh3 = mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad;
        }
        else if( aTh3 > mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad + ZERO_RAD_THD )
        {
            status = false;
        }
    }


    if( status )
    {
        if( aTh4 >= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad - ZERO_RAD_THD
         && aTh4 < mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad )
        {
            aTh4 = mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad;
        }
        else if( aTh4 < mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad - ZERO_RAD_THD )
        {
            status = false;
        }
    }
    if( status )
    {
        if( aTh4 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad + ZERO_RAD_THD
         && aTh4 > mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad )
        {
            aTh4 = mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad;
        }
        else if( aTh4 > mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad + ZERO_RAD_THD )
        {
            status = false;
        }
    }


    if( status )
    {
        if( aTh5 >= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.lowerAngleRad - ZERO_RAD_THD
         && aTh5 < mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.lowerAngleRad )
        {
            aTh5 = mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.lowerAngleRad;
        }
        else if( aTh5 < mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.lowerAngleRad - ZERO_RAD_THD )
        {
            status = false;
        }
    }
    if( status )
    {
        if( aTh5 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.upperAngleRad + ZERO_RAD_THD
         && aTh5 > mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.upperAngleRad )
        {
            aTh5 = mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.upperAngleRad;
        }
        else if( aTh5 > mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.upperAngleRad + ZERO_RAD_THD )
        {
            status = false;
        }
    }

    return status;
}


//!************************************************************************
//! Compute the inverse kinematic model for 4DOF
//! It is based on cascading a series of approaches that use analytical
//! solutions, so that provided results are exact.
//!
//! @returns true if the solution exists
//!************************************************************************
bool RobotArmKinematic::computeIkm
    (
    const double                aX,             //!< end-effector x coordinate
    const double                aY,             //!< end-effector y coordinate
    const double                aZ,             //!< end-effector z coordinate
    const double                aL1,            //!< length of base link
    const double                aL2,            //!< length of column link
    const double                aL3,            //!< length of upper arm link
    const double                aL4,            //!< length of lower arm link
    const double                aL5,            //!< length of end effector link
    std::vector<JointAngles>&   aJointAnglesVec //!< joint angles vector
    )
{
    bool status = false;

    double rank = 0;
    JointAngles angles;
    memset( &angles, 0, sizeof( angles ) );

    aJointAnglesVec.clear();
    double th2 = 0;
    double th3 = 0;
    double th4 = 0;
    double th5 = 0;

    // analytical solution @ th5 = -(th3+th4)
    if( computeIkmHee( aX, aY, aZ,
                       aL1, aL2, aL3, aL4, aL5,
                       th2, th3, th4, th5 ) )
    {
        if( validateIkmSet( th2, th3, th4, th5, aX, aY, aZ ) )
        {
            status = true;
            rank = computeJointAnglesRank( th2, th3, th4, th5 );
            angles = { th2, th3, th4, th5, rank };
            aJointAnglesVec.push_back( angles );
        }
    }

    // analytical solution @ th5 = 0
    if( computeIkmTh5Eq0( aX, aY, aZ,
                          aL1, aL2, aL3, aL4, aL5,
                          th2, th3, th4, th5 ) )
    {
        if( validateIkmSet( th2, th3, th4, th5, aX, aY, aZ ) )
        {
            status = true;
            rank = computeJointAnglesRank( th2, th3, th4, th5 );
            angles = { th2, th3, th4, th5, rank };
            aJointAnglesVec.push_back( angles );
        }
    }

    // analytical solution @ th5 = -th4
    if( computeIkmTh5EqMth4( aX, aY, aZ,
                             aL1, aL2, aL3, aL4, aL5,
                             th2, th3, th4, th5 ) )
    {
        if( validateIkmSet( th2, th3, th4, th5, aX, aY, aZ ) )
        {
            status = true;
            rank = computeJointAnglesRank( th2, th3, th4, th5 );
            angles = { th2, th3, th4, th5, rank };
            aJointAnglesVec.push_back( angles );
        }
    }

    // analytical solution @ th5 = -th3
    if( computeIkmTh5EqMth3( aX, aY, aZ,
                             aL1, aL2, aL3, aL4, aL5,
                             th2, th3, th4, th5 ) )
    {
        if( validateIkmSet( th2, th3, th4, th5, aX, aY, aZ ) )
        {
            status = true;
            rank = computeJointAnglesRank( th2, th3, th4, th5 );
            angles = { th2, th3, th4, th5, rank };
            aJointAnglesVec.push_back( angles );
        }
    }

    // analytical solution @ th4 = -th3
    if( computeIkmTh4EqMth3( aX, aY, aZ,
                             aL1, aL2, aL3, aL4, aL5,
                             th2, th3, th4, th5 ) )
    {
        if( validateIkmSet( th2, th3, th4, th5, aX, aY, aZ ) )
        {
            status = true;
            rank = computeJointAnglesRank( th2, th3, th4, th5 );
            angles = { th2, th3, th4, th5, rank };
            aJointAnglesVec.push_back( angles );
        }
    }

    // analytical solution @ th5+th4 = PI/2
    if( computeIkmTh5Th4EqPi2( aX, aY, aZ,
                               aL1, aL2, aL3, aL4, aL5,
                               th2, th3, th4, th5 ) )
    {
        if( validateIkmSet( th2, th3, th4, th5, aX, aY, aZ ) )
        {
            status = true;
            rank = computeJointAnglesRank( th2, th3, th4, th5 );
            angles = { th2, th3, th4, th5, rank };
            aJointAnglesVec.push_back( angles );
        }
    }

    // analytical solution @ th5+th4 = -PI/2
    if( computeIkmTh5Th4EqMpi2( aX, aY, aZ,
                                aL1, aL2, aL3, aL4, aL5,
                                th2, th3, th4, th5 ) )
    {
        if( validateIkmSet( th2, th3, th4, th5, aX, aY, aZ ) )
        {
            status = true;
            rank = computeJointAnglesRank( th2, th3, th4, th5 );
            angles = { th2, th3, th4, th5, rank };
            aJointAnglesVec.push_back( angles );
        }
    }

    // analytical solution @ th5+th4 = PI or
    //                       th5+th4 = -PI
    if( computeIkmTh5Th4EqPi( aX, aY, aZ,
                              aL1, aL2, aL3, aL4, aL5,
                              th2, th3, th4, th5 ) )
    {
        if( validateIkmSet( th2, th3, th4, th5, aX, aY, aZ ) )
        {
            status = true;
            rank = computeJointAnglesRank( th2, th3, th4, th5 );
            angles = { th2, th3, th4, th5, rank };
            aJointAnglesVec.push_back( angles );
        }
    }

    return status;
}


//!************************************************************************
//! Compute the inverse kinematic model for 4DOF
//! ***  Horizontal end-effector, th5 = -(th3+th4)  ***
//! It provides an exact analytical solution.
//! This function should be called from computeIkm()
//!
//! @returns true if the solution exists
//!************************************************************************
bool RobotArmKinematic::computeIkmHee
    (
    const double aX,    //!< end-effector x coordinate
    const double aY,    //!< end-effector y coordinate
    const double aZ,    //!< end-effector z coordinate
    const double aL1,   //!< length of base link
    const double aL2,   //!< length of column link
    const double aL3,   //!< length of upper arm link
    const double aL4,   //!< length of lower arm link
    const double aL5,   //!< length of end effector link
    double&      aTh2,  //!< angle of base joint [rad]
    double&      aTh3,  //!< angle of shoulder joint [rad]
    double&      aTh4,  //!< angle of elbow joint [rad]
    double&      aTh5   //!< angle of wrist joint [rad]
    ) const
{
    bool status = true;

    aTh2 = atan2( aY, aX );
    refineTh2Angle( aTh2 );

    aTh3 = 0;
    aTh4 = 0;
    aTh5 = 0;

    double th31 = 0;
    double th32 = 0;

    double th41 = 0;
    double th42 = 0;

    double a = 0;
    double b = 0;

    double u1 = 0;
    double u2 = 0;
    double u3 = 0;
    double u4 = 0;

    double v1 = 0;
    double v2 = 0;
    double v3 = 0;
    double v4 = 0;
    double v5 = 0;
    double v6 = 0;
    double v7 = 0;
    double v8 = 0;
    double v9 = 0;
    double v10 = 0;

    if( 0 == sin( aTh2 ) )
    {
        a = aX / cos( aTh2 ) - aL5;
    }
    else
    {
        a = aY / sin( aTh2 ) - aL5;
    }

    b = aZ - aL1 - aL2;

    // th4{1,2}
    u1 = a * a + b * b - aL3 * aL3 - aL4 * aL4;
    u2 =  u1 + 2 * aL3 * aL4;
    u3 = -u1 + 2 * aL3 * aL4;
    u4 = u2 * u3;

    if( u4 >= -ZERO_L_THD && u4 < 0 )
    {
        u4 = 0;
    }

    if( u4 < 0 )
    {
        status = false;
    }

    if( status )
    {
        th41 = -2 * atan2( sqrt( u4 ), u2 );
        th42 = -th41;

        refineAnglePlusMinusPi( th41 );
        refineAnglePlusMinusPi( th42 );
    }

    // th3{1,2}
    if( status )
    {
        v1 = a * a + b * b + aL3 * aL3 - aL4 * aL4 + 2 * a * aL3;
        v2 = a * a + b * b - aL3 * aL3 - aL4 * aL4 + 2 * aL3 * aL4;

        if( 0 == v2 )
        {
            status = false;
        }
    }

    if( status )
    {
        v3 = v2 * ( -a * a - b * b + aL3 * aL3 + aL4 * aL4 + 2 * aL3 * aL4 );

        if( v3 >= -ZERO_L_THD && v3 < 0 )
        {
            v3 = 0;
        }

        if( v3 < 0 )
        {
            status = false;
        }
    }

    if( status )
    {
        v3 = sqrt( v3 );
        double v32 = v3 / v2;
        v4 = aL4 * aL4 * v32;
        v5 = aL3 * aL3 * v32;
        v6 = b * b * v32;
        v7 = a * a * v32;
        v8 = 2 * aL3 * aL4 * v32;
        v9 = v7 + v6 - v5 - v4 + v8;
        v10 = 2 * b * aL3;

        th31 = 2 * atan2( v10 + v9, v1 );
        th32 = 2 * atan2( v10 - v9, v1 );

        refineAnglePlusMinusPi( th31 );
        refineAnglePlusMinusPi( th32 );

        double s1 = th31 + th41;
        double s2 = th32 + th42;

        if( ( ( s1 >= 0 ) && ( s1 <= 0.5 * SerialRobot::PI ) )
         || ( ( s1 >= -0.5 * SerialRobot::PI ) && ( s1 <= 0 ) )
          )
        {
            aTh3 = th31;
            aTh4 = th41;
        }
        else if( ( ( s2 >= 0 ) && ( s2 <= 0.5 * SerialRobot::PI ) )
              || ( ( s2 >= -0.5 * SerialRobot::PI ) && ( s2 <= 0 ) )
               )
        {
            aTh3 = th32;
            aTh4 = th42;
        }
        else
        {
            status = false;
        }

        aTh5 = -( aTh3 + aTh4 );
    }

    if( status )
    {
        status = checkJointAngles( aTh2, aTh3, aTh4, aTh5 );
    }

    return status;
}


//!************************************************************************
//! Compute the inverse kinematic model for 4DOF
//! ***  th5 = 0  ***
//! It provides an exact analytical solution.
//! This function should be called from computeIkm()
//!
//! @returns true if the solution exists
//!************************************************************************
bool RobotArmKinematic::computeIkmTh5Eq0
    (
    const double aX,    //!< end-effector x coordinate
    const double aY,    //!< end-effector y coordinate
    const double aZ,    //!< end-effector z coordinate
    const double aL1,   //!< length of base link
    const double aL2,   //!< length of column link
    const double aL3,   //!< length of upper arm link
    const double aL4,   //!< length of lower arm link
    const double aL5,   //!< length of end effector link
    double&      aTh2,  //!< angle of base joint [rad]
    double&      aTh3,  //!< angle of shoulder joint [rad]
    double&      aTh4,  //!< angle of elbow joint [rad]
    double&      aTh5   //!< angle of wrist joint [rad]
    ) const
{
    bool status = true;

    aTh2 = atan2( aY, aX );
    refineTh2Angle( aTh2 );

    aTh3 = 0;
    aTh4 = 0;
    aTh5 = 0;

    double th31 = 0;
    double th32 = 0;

    double th41 = 0;
    double th42 = 0;

    double a = 0;
    double b = 0;

    double u1 = 0;
    double u2 = 0;
    double u3 = 0;

    double v1 = 0;
    double v2 = 0;
    double v3 = 0;
    double v4 = 0;
    double v5 = 0;
    double v6 = 0;
    double v7 = 0;
    double v8 = 0;
    double v9 = 0;
    double v10 = 0;
    double v11 = 0;
    double v12 = 0;
    double v13 = 0;
    double v14 = 0;
    double v15 = 0;

    aTh5 = 0;

    if( 0 == sin( aTh2 ) )
    {
        a = aX / cos( aTh2 );
    }
    else
    {
        a = aY / sin( aTh2 );
    }

    b = aZ - aL1 - aL2;

    // th4{1,2}
    u1 = a * a + b * b - aL3 * aL3 - aL4 * aL4 - aL5 * aL5 - 2 * aL4 * aL5;
    u2 = 2 * aL3 * ( aL4 + aL5 );
    u3 = ( u1 + u2 ) * ( -u1 + u2 );

    if( u3 >= -ZERO_L_THD && u3 < 0 )
    {
        u3 = 0;
    }

    if( u3 < 0 )
    {
        status = false;
    }

    if( status )
    {
        th41 = 2 * atan2( sqrt( u3 ), u1 + u2 );
        th42 = -th41;

        refineAnglePlusMinusPi( th41 );
        refineAnglePlusMinusPi( th42 );
    }

    // th3{1,2}
    if( status )
    {
        v1 = a * a + b * b + aL3 * aL3 - aL4 * aL4 - aL5 * aL5 - 2 * aL4 * aL5 + 2 * a * aL3;
        v2 = a * a + b * b - aL3 * aL3 - aL4 * aL4 - aL5 * aL5 - 2 * aL4 * aL5;
        v3 = 2 * aL3 * ( aL4 + aL5 );
        v4 = v2 + v3;

        if( 0 == v4 )
        {
            status = false;
        }
    }

    double vprod = v4 * ( -v2 + v3 );

    if( vprod >= -ZERO_L_THD && vprod < 0 )
    {
        vprod = 0;
    }

    if( status )
    {
        if( vprod < 0 )
        {
            status = false;
        }
    }

    if( status )
    {
        v5 = sqrt( vprod );
        double v54 = v5 / v4;
        v6 = aL5 * aL5 * v54;
        v7 = aL4 * aL4 * v54;
        v8 = aL3 * aL3 * v54;
        v9 = b * b * v54;
        v10 = a * a * v54;
        v11 = 2 * aL4 * aL5 * v54;
        v12 = 2 * aL3 * aL5 * v54;
        v13 = 2 * aL3 * aL4 * v54;
        v14 = v10 + v9 - v8 - v7 - v6 + v13 + v12 - v11;
        v15 = 2 * b * aL3;

        th31 = 2 * atan2( v15 - v14, v1 );
        th32 = 2 * atan2( v15 + v14, v1 );

        refineAnglePlusMinusPi( th31 );
        refineAnglePlusMinusPi( th32 );

        if( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad <= th31
            && th31 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad
            &&
            mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad <= th41
            && th41 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad
          )
        {
            aTh3 = th31;
            aTh4 = th41;
        }
        else if( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad <= th32
                 && th32 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad
                 &&
                 mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad <= th42
                 && th42 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad
               )
        {
            aTh3 = th32;
            aTh4 = th42;
        }
        else
        {
            status = false;
        }
    }

    if( status )
    {
        status = checkJointAngles( aTh2, aTh3, aTh4, aTh5 );
    }

    return status;
}


//!************************************************************************
//! Compute the inverse kinematic model for 4DOF
//! ***  th5 = -th4  ***
//! It provides an exact analytical solution.
//! This function should be called from computeIkm()
//!
//! @returns true if the solution exists
//!************************************************************************
bool RobotArmKinematic::computeIkmTh5EqMth4
    (
    const double aX,    //!< end-effector x coordinate
    const double aY,    //!< end-effector y coordinate
    const double aZ,    //!< end-effector z coordinate
    const double aL1,   //!< length of base link
    const double aL2,   //!< length of column link
    const double aL3,   //!< length of upper arm link
    const double aL4,   //!< length of lower arm link
    const double aL5,   //!< length of end effector link
    double&      aTh2,  //!< angle of base joint [rad]
    double&      aTh3,  //!< angle of shoulder joint [rad]
    double&      aTh4,  //!< angle of elbow joint [rad]
    double&      aTh5   //!< angle of wrist joint [rad]
    ) const
{
    bool status = true;

    aTh2 = atan2( aY, aX );
    refineTh2Angle( aTh2 );

    aTh3 = 0;
    aTh4 = 0;
    aTh5 = 0;

    double th31 = 0;
    double th32 = 0;

    double th41 = 0;
    double th42 = 0;

    double th51 = 0;
    double th52 = 0;

    double a = 0;
    double b = 0;

    double u1 = 0;
    double u2 = 0;
    double u3 = 0;

    double v1 = 0;
    double v2 = 0;
    double v3 = 0;
    double v4 = 0;
    double v5 = 0;
    double v6 = 0;
    double v7 = 0;
    double v8 = 0;
    double v9 = 0;
    double v10 = 0;
    double v11 = 0;
    double v12 = 0;
    double v13 = 0;
    double v14 = 0;
    double v15 = 0;

    if( 0 == sin( aTh2 ) )
    {
        a = aX / cos( aTh2 );
    }
    else
    {
        a = aY / sin( aTh2 );
    }

    b = aZ - aL1 - aL2;

    // th4{1,2}
    u1 = a * a + b * b - aL3 * aL3 - aL4 * aL4 - aL5 * aL5 - 2 * aL3 * aL5;
    u2 = 2 * aL4 * ( aL3 + aL5 );
    u3 = ( u1 + u2 ) * ( -u1 + u2 );

    if( u3 >= -ZERO_L_THD && u3 < 0 )
    {
        u3 = 0;
    }

    if( u3 < 0 )
    {
        status = false;
    }

    if( status )
    {
        th41 = 2 * atan2( sqrt( u3 ), u1 + u2 );
        th42 = -th41;

        refineAnglePlusMinusPi( th41 );
        refineAnglePlusMinusPi( th42 );

        th51 = -th41;
        th52 = -th42;
    }

    // th3{1,2}
    if( status )
    {
        v1 = a * a + b * b + aL3 * aL3 - aL4 * aL4 + aL5 * aL5 + 2 * aL3 * aL5 + 2 * a * ( aL3 + aL5 );
        v2 = a * a + b * b - aL3 * aL3 - aL4 * aL4 - aL5 * aL5 - 2 * aL3 * aL5;
        v3 = 2 * aL4 * ( aL3 + aL5 );
        v4 = v2 + v3;    

        if( 0 == v4 )
        {
            status = false;
        }
    }

    double vprod = v4 * ( -v2 + v3 );

    if( vprod >= -ZERO_L_THD && vprod < 0 )
    {
        vprod = 0;
    }

    if( status )
    {
        if( vprod < 0 )
        {
            status = false;
        }
    }

    if( status )
    {
        v5 = sqrt( vprod );
        double v54 = v5 / v4;
        v6 = aL5 * aL5 * v54;
        v7 = aL4 * aL4 * v54;
        v8 = aL3 * aL3 * v54;
        v9 = b * b * v54;
        v10 = a * a * v54;
        v11 = 2 * aL4 * aL5 * v54;
        v12 = 2 * aL3 * aL5 * v54;
        v13 = 2 * aL3 * aL4 * v54;
        v14 = v10 + v9 - v8 - v7 - v6 + v13 + v12 - v11;
        v15 = 2 * b * ( aL3 + aL5 );

        th31 = 2 * atan2( v15 - v14, v1 );
        th32 = 2 * atan2( v15 + v14, v1 );

        refineAnglePlusMinusPi( th31 );
        refineAnglePlusMinusPi( th32 );

        if( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad <= th31
            && th31 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad
            &&
            mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad <= th41
            && th41 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad
            &&
            mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.lowerAngleRad <= th51
            && th51 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.upperAngleRad
          )
        {
            aTh3 = th31;
            aTh4 = th41;
            aTh5 = th51;
        }
        else if( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad <= th32
                 && th32 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad
                 &&
                 mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad <= th42
                 && th42 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad
                 &&
                 mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.lowerAngleRad <= th52
                 && th52 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.upperAngleRad
               )
        {
            aTh3 = th32;
            aTh4 = th42;
            aTh5 = th52;
        }
        else
        {
            status = false;
        }
    }

    if( status )
    {
        status = checkJointAngles( aTh2, aTh3, aTh4, aTh5 );
    }

    return status;
}


//!************************************************************************
//! Compute the inverse kinematic model for 4DOF
//! ***  th5 = -th3  ***
//! It provides an exact analytical solution.
//! This function should be called from computeIkm()
//!
//! @returns true if the solution exists
//!************************************************************************
bool RobotArmKinematic::computeIkmTh5EqMth3
    (
    const double aX,    //!< end-effector x coordinate
    const double aY,    //!< end-effector y coordinate
    const double aZ,    //!< end-effector z coordinate
    const double aL1,   //!< length of base link
    const double aL2,   //!< length of column link
    const double aL3,   //!< length of upper arm link
    const double aL4,   //!< length of lower arm link
    const double aL5,   //!< length of end effector link
    double&      aTh2,  //!< angle of base joint [rad]
    double&      aTh3,  //!< angle of shoulder joint [rad]
    double&      aTh4,  //!< angle of elbow joint [rad]
    double&      aTh5   //!< angle of wrist joint [rad]
    ) const
{
    bool status = true;

    aTh2 = atan2( aY, aX );
    refineTh2Angle( aTh2 );

    aTh3 = 0;
    aTh4 = 0;
    aTh5 = 0;

    double th31 = 0;
    double th32 = 0;

    double th41 = 0;
    double th42 = 0;

    double th51 = 0;
    double th52 = 0;

    double a = 0;
    double b = 0;

    double u1 = 0;
    double u2 = 0;
    double u3 = 0;
    double u4 = 0;
    double u5 = 0;

    double v1 = 0;
    double v2 = 0;
    double v3 = 0;
    double v4 = 0;
    double v5 = 0;
    double v6 = 0;
    double v7 = 0;
    double v8 = 0;
    double v9 = 0;
    double v10 = 0;
    double v11 = 0;
    double v12 = 0;
    double v13 = 0;
    double v14 = 0;
    double v15 = 0;
    double v16 = 0;
    double v17 = 0;
    double v18 = 0;
    double v19 = 0;
    double v20 = 0;
    double v21 = 0;
    double v22 = 0;

    if( 0 == sin( aTh2 ) )
    {
        a = aX / cos( aTh2 );
    }
    else
    {
        a = aY / sin( aTh2 );
    }

    b = aZ - aL1 - aL2;

    // th4{1,2}
    u1 = a * a + b * b - aL3 * aL3 - aL4 * aL4 + aL5 * aL5 + 2 * ( a * aL5 + aL3 * aL4 );
    u2 = 2 * ( aL3 * aL3 + aL4 * aL4 + aL5 * aL5 );
    u3 = a * a * ( -a * a - 2 * b * b + u2 ) + b * b * ( -b * b + u2 ) + 8 * a * aL3 * aL4 * aL5
            - pow( aL3, 4.0 ) - pow( aL4, 4.0 ) - pow( aL5, 4.0 )
            + 2 * ( pow( aL3 * aL4, 2.0 ) + pow( aL3 * aL5, 2.0 ) + pow( aL4 * aL5, 2.0 ) );

    if( u3 >= -ZERO_L_THD && u3 < 0 )
    {
        u3 = 0;
    }

    if( u3 < 0 )
    {
        status = false;
    }

    if( status )
    {
        u4 = sqrt( u3 );
        u5 = 2 * b * aL5;

        th41 = 2 * atan2( u5 - u4, u1 );
        th41 = 2 * atan2( u5 + u4, u1 );

        refineAnglePlusMinusPi( th41 );
        refineAnglePlusMinusPi( th42 );
    }

    // th3{1,2}
    if( status )
    {
        v1 = a * a + b * b - aL3 * aL3 - aL4 * aL4 + aL5 * aL5 + 2 * ( a * aL5 + aL3 * aL4 );

        if( 0 == v1 )
        {
            status = false;
        }
    }

    if( status )
    {
        v2 = a * a + b * b + aL3 * aL3 - aL4 * aL4 - aL5 * aL5 + 2 * ( a * aL3 + aL4 * aL5 );
        v3 = u3;
        v4 = sqrt( v3 );
        v5 = u5;
        v6 = v5 - v4;
        v7 = v5 + v4;

        double v61 = v6 / v1;
        double v71 = v7 / v1;

        v8 = a * a * v71;
        v9 = b * b * v71;
        v10 = aL3 * aL3 * v71;
        v11 = aL4 * aL4 * v71;
        v12 = aL5 * aL5 * v71;
        v13 = 2 * a * aL5 * v71;
        v14 = 2 * aL3 * aL4 * v71;

        v15 = a * a * v61;
        v16 = b * b * v61;
        v17 = aL3 * aL3 * v61;
        v18 = aL4 * aL4 * v61;
        v19 = aL5 * aL5 * v61;
        v20 = 2 * a * aL5 * v61;
        v21 = 2 * aL3 * aL4 * v61;

        v22 = 2 * b * ( aL3 + aL5 );

        th31 = -2 * atan2( v15 + v16 - v17 - v18 + v19 + v20 + v21 - v22, v2 );
        th32 = -2 * atan2( v8 + v9 - v10 - v11 + v12 + v13 + v14 - v22, v2 );

        refineAnglePlusMinusPi( th31 );
        refineAnglePlusMinusPi( th32 );

        th51 = -th31;
        th52 = -th32;

        if( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad <= th31
            && th31 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad
            &&
            mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad <= th41
            && th41 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad
            &&
            mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.lowerAngleRad <= th51
            && th51 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.upperAngleRad
          )
        {
            aTh3 = th31;
            aTh4 = th41;
            aTh5 = th51;
        }
        else if( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad <= th32
                 && th32 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad
                 &&
                 mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad <= th42
                 && th42 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad
                 &&
                 mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.lowerAngleRad <= th52
                 && th52 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.upperAngleRad
               )
        {
            aTh3 = th32;
            aTh4 = th42;
            aTh5 = th52;
        }
        else
        {
            status = false;
        }
    }

    if( status )
    {
        status = checkJointAngles( aTh2, aTh3, aTh4, aTh5 );
    }

    return status;
}


//!************************************************************************
//! Compute the inverse kinematic model for 4DOF
//! ***  th4 = -th3  ***
//! It provides an exact analytical solution.
//! This function should be called from computeIkm()
//!
//! @returns true if the solution exists
//!************************************************************************
bool RobotArmKinematic::computeIkmTh4EqMth3
    (
    const double aX,    //!< end-effector x coordinate
    const double aY,    //!< end-effector y coordinate
    const double aZ,    //!< end-effector z coordinate
    const double aL1,   //!< length of base link
    const double aL2,   //!< length of column link
    const double aL3,   //!< length of upper arm link
    const double aL4,   //!< length of lower arm link
    const double aL5,   //!< length of end effector link
    double&      aTh2,  //!< angle of base joint [rad]
    double&      aTh3,  //!< angle of shoulder joint [rad]
    double&      aTh4,  //!< angle of elbow joint [rad]
    double&      aTh5   //!< angle of wrist joint [rad]
    ) const
{
    bool status = true;

    aTh2 = atan2( aY, aX );
    refineTh2Angle( aTh2 );

    aTh3 = 0;
    aTh4 = 0;
    aTh5 = 0;

    double th31 = 0;
    double th32 = 0;

    double th41 = 0;
    double th42 = 0;

    double th51 = 0;
    double th52 = 0;

    double a = 0;
    double b = 0;

    double u1 = 0;
    double u2 = 0;
    double u3 = 0;
    double u4 = 0;
    double u5 = 0;
    double u6 = 0;

    double v1 = 0;
    double v2 = 0;
    double v3 = 0;
    double v4 = 0;
    double v5 = 0;
    double v6 = 0;
    double v7 = 0;
    double v8 = 0;
    double v9 = 0;
    double v10 = 0;
    double v11 = 0;
    double v12 = 0;
    double v13 = 0;
    double v14 = 0;
    double v15 = 0;
    double v16 = 0;
    double v17 = 0;
    double v18 = 0;
    double v19 = 0;
    double v20 = 0;
    double v21 = 0;
    double v22 = 0;

    if( 0 == sin( aTh2 ) )
    {
        a = aX / cos( aTh2 ) - aL4;
    }
    else
    {
        a = aY / sin( aTh2 ) - aL4;
    }

    b = aZ - aL1 - aL2;

    // th5{1,2}
    u1 = a * a + b * b - aL3 * aL3 - aL5 * aL5;
    u2 = 2 * aL3 * aL5;
    u3 = ( u1 + u2 ) * ( -u1 + u2 );

    if( u3 >= -ZERO_L_THD && u3 < 0 )
    {
        u3 = 0;
    }

    if( u3 < 0 )
    {
        status = false;
    }

    if( status )
    {
        u4 = sqrt( u3 );
        u5 = 2 * b * aL5;
        u6 = a * a + b * b - aL3 * aL3 + aL5 * aL5 + 2 * a * aL5;

        th51 = 2 * atan2( u5 + u4, u6 );
        th51 = 2 * atan2( u5 - u4, u6 );

        refineAnglePlusMinusPi( th51 );
        refineAnglePlusMinusPi( th52 );
    }

    // th3{1,2}
    if( status )
    {
        v1 = a * a + b * b + 2 * a * aL5;
        v2 = aL3 * aL3 - aL5 * aL5;
        v3 = v1 - v2;

        if( 0 == v3 )
        {
            status = false;
        }
    }

    if( status )
    {
        v4 = v1 + v2;
        v5 = u3;
        v6 = sqrt( v5 );
        v7 = u5;
        v8 = v7 - v6;
        v9 = v7 + v6;

        double v83 = v8 / v3;
        double v93 = v9 / v3;

        v10 = a * a * v83;
        v11 = b * b * v83;
        v12 = aL3 * aL3 * v83;
        v13 = aL5 * aL5 * v83;
        v14 = 2 * a * aL5 * v83;

        v15 = a * a * v93;
        v16 = b * b * v93;
        v17 = aL3 * aL3 * v93;
        v18 = aL5 * aL5 * v93;
        v19 = 2 * a * aL5 * v93;

        v20 = 2 * b * ( aL3 + aL5 );

        th31 = -2 * atan2( v15 + v16 - v17 + v18 + v19 - v20, v4 );
        th32 = -2 * atan2( v10 + v11 - v12 + v13 + v14 - v20, v4 );

        refineAnglePlusMinusPi( th31 );
        refineAnglePlusMinusPi( th32 );

        th41 = -th31;
        th42 = -th32;

        if( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad <= th31
            && th31 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad
            &&
            mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad <= th41
            && th41 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad
            &&
            mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.lowerAngleRad <= th51
            && th51 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.upperAngleRad
          )
        {
            aTh3 = th31;
            aTh4 = th41;
            aTh5 = th51;
        }
        else if( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad <= th32
                 && th32 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad
                 &&
                 mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad <= th42
                 && th42 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad
                 &&
                 mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.lowerAngleRad <= th52
                 && th52 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.upperAngleRad
               )
        {
            aTh3 = th32;
            aTh4 = th42;
            aTh5 = th52;
        }
        else
        {
            status = false;
        }
    }

    if( status )
    {
        status = checkJointAngles( aTh2, aTh3, aTh4, aTh5 );
    }

    return status;
}


//!************************************************************************
//! Compute the inverse kinematic model for 4DOF
//! ***  th5+th4 = PI/2  ***
//! It provides an exact analytical solution.
//! This function should be called from computeIkm()
//!
//! @returns true if the solution exists
//!************************************************************************
bool RobotArmKinematic::computeIkmTh5Th4EqPi2
    (
    const double aX,    //!< end-effector x coordinate
    const double aY,    //!< end-effector y coordinate
    const double aZ,    //!< end-effector z coordinate
    const double aL1,   //!< length of base link
    const double aL2,   //!< length of column link
    const double aL3,   //!< length of upper arm link
    const double aL4,   //!< length of lower arm link
    const double aL5,   //!< length of end effector link
    double&      aTh2,  //!< angle of base joint [rad]
    double&      aTh3,  //!< angle of shoulder joint [rad]
    double&      aTh4,  //!< angle of elbow joint [rad]
    double&      aTh5   //!< angle of wrist joint [rad]
    ) const
{
    bool status = true;

    aTh2 = atan2( aY, aX );
    refineTh2Angle( aTh2 );

    aTh3 = 0;
    aTh4 = 0;
    aTh5 = 0;

    double th31 = 0;
    double th32 = 0;

    double th41 = 0;
    double th42 = 0;

    double th51 = 0;
    double th52 = 0;

    double a = 0;
    double b = 0;

    double u1 = 0;
    double u2 = 0;
    double u3 = 0;
    double u4 = 0;
    double u5 = 0;
    double u6 = 0;
    double u7 = 0;
    double u8 = 0;

    double v1 = 0;
    double v2 = 0;
    double v3 = 0;
    double v4 = 0;
    double v5 = 0;
    double v6 = 0;
    double v7 = 0;
    double v8 = 0;
    double v9 = 0;
    double v10 = 0;
    double v11 = 0;
    double v12 = 0;
    double v13 = 0;
    double v14 = 0;
    double v15 = 0;
    double v16 = 0;
    double v17 = 0;
    double v18 = 0;

    if( 0 == sin( aTh2 ) )
    {
        a = aX / cos( aTh2 );
    }
    else
    {
        a = aY / sin( aTh2 );
    }

    b = aZ - aL1 - aL2;

    // th4{1,2}
    u1 = a * a + b * b - aL3 * aL3 - aL4 * aL4 - aL5 * aL5 + 2 * aL3 * aL4;
    u2 = aL3 * aL3 + aL4 * aL4 + aL5 * aL5;
    u3 = aL3 * aL3 - 2 * ( aL4 * aL4 - aL5 * aL5 );
    u4 = aL4 * aL4 - aL5 * aL5;
    u5 = a * a * ( 2 * u2 - a * a - 2 * b * b ) + b * b * ( 2 * u2 - b * b ) - aL3 * aL3 * u3 - u4 * u4;

    if( u5 >= -ZERO_L_THD && u5 < 0 )
    {
        u5 = 0;
    }

    if( u5 < 0 )
    {
        status = false;
    }

    if( status )
    {
        u6 = sqrt( u5 );
        u7 = 2 * aL4 * aL5 - u6;
        u8 = 2 * aL4 * aL5 + u6;

        th41 = 2 * atan2( u7, u1 );
        th42 = 2 * atan2( u8, u1 );

        refineAnglePlusMinusPi( th41 );
        refineAnglePlusMinusPi( th42 );

        th51 = 0.5 * SerialRobot::PI - th41;
        th52 = 0.5 * SerialRobot::PI - th42;

        refineAnglePlusMinusPi( th51 );
        refineAnglePlusMinusPi( th52 );
    }

    // th3{1,2}
    if( status )
    {
        v1 = u6;
        v2 = u7;
        v3 = u8;
        v4 = a * a + b * b - aL3 * aL3 - aL4 * aL4 - aL5 * aL5 + 2 * aL3 * aL4;
        v5 = a * a + b * b + aL3 * aL3 - aL4 * aL4 + aL5 * aL5 + 2 * ( a * aL3 + b * aL5 );

        if( 0 == v4 )
        {
            status = false;
        }
    }

    if( status )
    {
        double v34 = v3 / v4;
        double v24 = v2 / v4;

        v6 = aL3 * aL3 * v34;
        v7 = aL4 * aL4 * v34;
        v8 = aL5 * aL5 * v34;
        v9 = a * a * v34;
        v10 = b * b * v34;
        v11 = 2 * aL3 * aL4 * v34;

        v12 = aL3 * aL3 * v24;
        v13 = aL4 * aL4 * v24;
        v14 = aL5 * aL5 * v24;
        v15 = a * a * v24;
        v16 = b * b * v24;
        v17 = 2 * aL3 * aL4 * v24;

        v18 = 2 * ( b * aL3 - a * aL5 + aL4 * aL5 );

        th31 = 2 * atan2( v18 + v12 + v13 + v14 - v15 - v16 - v17, v5 );
        th32 = 2 * atan2( v18 + v6 + v7 + v8 - v9 - v10 - v11, v5 );

        refineAnglePlusMinusPi( th31 );
        refineAnglePlusMinusPi( th32 );

        if( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad <= th31
            && th31 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad
            &&
            mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad <= th41
            && th41 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad
            &&
            mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.lowerAngleRad <= th51
            && th51 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.upperAngleRad
          )
        {
            aTh3 = th31;
            aTh4 = th41;
            aTh5 = th51;
        }
        else if( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad <= th32
                 && th32 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad
                 &&
                 mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad <= th42
                 && th42 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad
                 &&
                 mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.lowerAngleRad <= th52
                 && th52 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.upperAngleRad
               )
        {
            aTh3 = th32;
            aTh4 = th42;
            aTh5 = th52;
        }
        else
        {
            status = false;
        }
    }

    if( status )
    {
        status = checkJointAngles( aTh2, aTh3, aTh4, aTh5 );
    }

    return status;
}


//!************************************************************************
//! Compute the inverse kinematic model for 4DOF
//! ***  th5+th4 = -PI/2  ***
//! It provides an exact analytical solution.
//! This function should be called from computeIkm()
//!
//! @returns true if the solution exists
//!************************************************************************
bool RobotArmKinematic::computeIkmTh5Th4EqMpi2
    (
    const double aX,    //!< end-effector x coordinate
    const double aY,    //!< end-effector y coordinate
    const double aZ,    //!< end-effector z coordinate
    const double aL1,   //!< length of base link
    const double aL2,   //!< length of column link
    const double aL3,   //!< length of upper arm link
    const double aL4,   //!< length of lower arm link
    const double aL5,   //!< length of end effector link
    double&      aTh2,  //!< angle of base joint [rad]
    double&      aTh3,  //!< angle of shoulder joint [rad]
    double&      aTh4,  //!< angle of elbow joint [rad]
    double&      aTh5   //!< angle of wrist joint [rad]
    ) const
{
    bool status = true;

    aTh2 = atan2( aY, aX );
    refineTh2Angle( aTh2 );

    aTh3 = 0;
    aTh4 = 0;
    aTh5 = 0;

    double th31 = 0;
    double th32 = 0;

    double th41 = 0;
    double th42 = 0;

    double th51 = 0;
    double th52 = 0;

    double a = 0;
    double b = 0;

    double u1 = 0;
    double u2 = 0;
    double u3 = 0;
    double u4 = 0;
    double u5 = 0;
    double u6 = 0;
    double u7 = 0;
    double u8 = 0;

    double v1 = 0;
    double v2 = 0;
    double v3 = 0;
    double v4 = 0;
    double v5 = 0;
    double v6 = 0;
    double v7 = 0;
    double v8 = 0;
    double v9 = 0;
    double v10 = 0;
    double v11 = 0;
    double v12 = 0;
    double v13 = 0;
    double v14 = 0;
    double v15 = 0;
    double v16 = 0;
    double v17 = 0;
    double v18 = 0;

    if( 0 == sin( aTh2 ) )
    {
        a = aX / cos( aTh2 );
    }
    else
    {
        a = aY / sin( aTh2 );
    }

    b = aZ - aL1 - aL2;

    // th4{1,2}
    u1 = a * a + b * b - aL3 * aL3 - aL4 * aL4 - aL5 * aL5 + 2 * aL3 * aL4;
    u2 = aL3 * aL3 + aL4 * aL4 + aL5 * aL5;
    u3 = aL3 * aL3 - 2 * ( aL4 * aL4 - aL5 * aL5 );
    u4 = aL4 * aL4 - aL5 * aL5;
    u5 = a * a * ( 2 * u2 - a * a - 2 * b * b ) + b * b * ( 2 * u2 - b * b ) - aL3 * aL3 * u3 - u4 * u4;

    if( u5 >= -ZERO_L_THD && u5 < 0 )
    {
        u5 = 0;
    }

    if( u5 < 0 )
    {
        status = false;
    }

    if( status )
    {
        u6 = sqrt( u5 );
        u7 = 2 * aL4 * aL5 - u6;
        u8 = 2 * aL4 * aL5 + u6;

        th41 = -2 * atan2( u7, u1 );
        th42 = -2 * atan2( u8, u1 );

        refineAnglePlusMinusPi( th41 );
        refineAnglePlusMinusPi( th42 );

        th51 = -0.5 * SerialRobot::PI - th41;
        th52 = -0.5 * SerialRobot::PI - th42;

        refineAnglePlusMinusPi( th51 );
        refineAnglePlusMinusPi( th52 );
    }

    // th3{1,2}
    if( status )
    {
        v1 = u6;
        v2 = u7;
        v3 = u8;
        v4 = a * a + b * b - aL3 * aL3 - aL4 * aL4 - aL5 * aL5 + 2 * aL3 * aL4;
        v5 = a * a + b * b + aL3 * aL3 - aL4 * aL4 + aL5 * aL5 + 2 * ( a * aL3 - b * aL5 );    

        if( 0 == v4 )
        {
            status = false;
        }
    }

    if( status )
    {
        double v34 = v3 / v4;
        double v24 = v2 / v4;

        v6 = aL3 * aL3 * v34;
        v7 = aL4 * aL4 * v34;
        v8 = aL5 * aL5 * v34;
        v9 = a * a * v34;
        v10 = b * b * v34;
        v11 = 2 * aL3 * aL4 * v34;

        v12 = aL3 * aL3 * v24;
        v13 = aL4 * aL4 * v24;
        v14 = aL5 * aL5 * v24;
        v15 = a * a * v24;
        v16 = b * b * v24;
        v17 = 2 * aL3 * aL4 * v24;

        v18 = 2 * ( b * aL3 + a * aL5 - aL4 * aL5 );

        th31 = 2 * atan2( v18 - ( v12 + v13 + v14 - v15 - v16 - v17 ), v5 );
        th32 = 2 * atan2( v18 - ( v6 + v7 + v8 - v9 - v10 - v11 ), v5 );

        refineAnglePlusMinusPi( th31 );
        refineAnglePlusMinusPi( th32 );

        if( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad <= th31
            && th31 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad
            &&
            mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad <= th41
            && th41 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad
            &&
            mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.lowerAngleRad <= th51
            && th51 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.upperAngleRad
          )
        {
            aTh3 = th31;
            aTh4 = th41;
            aTh5 = th51;
        }
        else if( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad <= th32
                 && th32 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad
                 &&
                 mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad <= th42
                 && th42 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad
                 &&
                 mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.lowerAngleRad <= th52
                 && th52 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.upperAngleRad
               )
        {
            aTh3 = th32;
            aTh4 = th42;
            aTh5 = th52;
        }
        else
        {
            status = false;
        }
    }

    if( status )
    {
        status = checkJointAngles( aTh2, aTh3, aTh4, aTh5 );
    }

    return status;
}


//!************************************************************************
//! Compute the inverse kinematic model for 4DOF
//! ***  th5+th4 = PI  ***
//!  OR
//! ***  th5+th4 = -PI  ***
//! It provides an exact analytical solution.
//! This function should be called from computeIkm()
//!
//! @returns true if the solution exists
//!************************************************************************
bool RobotArmKinematic::computeIkmTh5Th4EqPi
    (
    const double aX,    //!< end-effector x coordinate
    const double aY,    //!< end-effector y coordinate
    const double aZ,    //!< end-effector z coordinate
    const double aL1,   //!< length of base link
    const double aL2,   //!< length of column link
    const double aL3,   //!< length of upper arm link
    const double aL4,   //!< length of lower arm link
    const double aL5,   //!< length of end effector link
    double&      aTh2,  //!< angle of base joint [rad]
    double&      aTh3,  //!< angle of shoulder joint [rad]
    double&      aTh4,  //!< angle of elbow joint [rad]
    double&      aTh5   //!< angle of wrist joint [rad]
    ) const
{
    bool status = true;

    aTh2 = atan2( aY, aX );
    refineTh2Angle( aTh2 );

    aTh3 = 0;
    aTh4 = 0;
    aTh5 = 0;

    double th31 = 0;
    double th32 = 0;

    double th41 = 0;
    double th42 = 0;

    double th51 = 0;
    double th52 = 0;

    double a = 0;
    double b = 0;

    double u1 = 0;
    double u2 = 0;
    double u3 = 0;

    double v1 = 0;
    double v2 = 0;
    double v3 = 0;
    double v4 = 0;
    double v5 = 0;
    double v6 = 0;
    double v7 = 0;
    double v8 = 0;
    double v9 = 0;
    double v10 = 0;
    double v11 = 0;
    double v12 = 0;
    double v13 = 0;
    double v14 = 0;
    double v15 = 0;

    if( 0 == sin( aTh2 ) )
    {
        a = aX / cos( aTh2 );
    }
    else
    {
        a = aY / sin( aTh2 );
    }

    b = aZ - aL1 - aL2;

    // th4{1,2}
    u1 = a * a + b * b - aL3 * aL3 - aL4 * aL4 - aL5 * aL5 + 2 * aL3 * aL5;
    u2 = 2 * aL4 * ( aL3 - aL5 );
    u3 = ( u1 + u2 ) * ( -u1 + u2 );

    if( u3 >= -ZERO_L_THD && u3 < 0 )
    {
        u3 = 0;
    }

    if( u3 < 0 )
    {
        status = false;
    }

    if( status )
    {
        th41 = 2 * atan2( sqrt( u3 ), u1 + u2 );
        th42 = -th41;

        refineAnglePlusMinusPi( th41 );
        refineAnglePlusMinusPi( th42 );
    }

    // th3{1,2}
    if( status )
    {
        v1 = a * a + b * b + aL3 * aL3 - aL4 * aL4 + aL5 * aL5 - 2 * aL3 * aL5 + 2 * a * ( aL3 - aL5 );
        v2 = a * a + b * b - aL3 * aL3 - aL4 * aL4 - aL5 * aL5 + 2 * aL3 * aL5;
        v3 = 2 * aL4 * ( aL3 - aL5 );
        v4 = v2 + v3;    

        if( 0 == v4 )
        {
            status = false;
        }
    }

    double vprod = v4 * ( -v2 + v3 );

    if( vprod >= -ZERO_L_THD && vprod < 0 )
    {
        vprod = 0;
    }

    if( status )
    {
        if( vprod < 0 )
        {
            status = false;
        }
    }

    if( status )
    {
        v5 = sqrt( vprod );
        double v54 = v5 / v4;
        v6 = aL5 * aL5 * v54;
        v7 = aL4 * aL4 * v54;
        v8 = aL3 * aL3 * v54;
        v9 = b * b * v54;
        v10 = a * a * v54;
        v11 = 2 * aL4 * aL5 * v54;
        v12 = 2 * aL3 * aL5 * v54;
        v13 = 2 * aL3 * aL4 * v54;
        v14 = v10 + v9 - v8 - v7 - v6 + v13 + v12 - v11;
        v15 = 2 * b * ( aL3 - aL5 );

        th31 = 2 * atan2( v15 - v14, v1 );
        th32 = 2 * atan2( v15 + v14, v1 );

        refineAnglePlusMinusPi( th31 );
        refineAnglePlusMinusPi( th32 );

        bool casePlusPi = false;
        bool caseMinusPi = false;

        if( !casePlusPi )
        {
            th51 = SerialRobot::PI - th41;
            th52 = SerialRobot::PI - th42;

            refineAnglePlusMinusPi( th51 );
            refineAnglePlusMinusPi( th52 );

            refineAnglesPlusPi( th41, th51 );
            refineAnglesPlusPi( th42, th52 );

            if( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad <= th31
                && th31 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad
                &&
                mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad <= th41
                && th41 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad
                &&
                mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.lowerAngleRad <= th51
                && th51 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.upperAngleRad
              )
            {
                aTh3 = th31;
                aTh4 = th41;
                aTh5 = th51;

                casePlusPi = true;
            }
            else if( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad <= th32
                     && th32 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad
                     &&
                     mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad <= th42
                     && th42 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad
                     &&
                     mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.lowerAngleRad <= th52
                     && th52 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.upperAngleRad
                   )
            {
                aTh3 = th32;
                aTh4 = th42;
                aTh5 = th52;

                casePlusPi = true;
            }
        }

        if( !casePlusPi )
        {
            // case II
            th51 = -SerialRobot::PI - th41;
            th52 = -SerialRobot::PI - th42;

            refineAnglePlusMinusPi( th51 );
            refineAnglePlusMinusPi( th52 );

            refineAnglesMinusPi( th41, th51 );
            refineAnglesMinusPi( th42, th52 );

            if( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad <= th31
                && th31 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad
                &&
                mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad <= th41
                && th41 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad
                &&
                mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.lowerAngleRad <= th51
                && th51 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.upperAngleRad
              )
            {
                aTh3 = th31;
                aTh4 = th41;
                aTh5 = th51;

                caseMinusPi = true;
            }
            else if( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad <= th32
                     && th32 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad
                     &&
                     mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad <= th42
                     && th42 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad
                     &&
                     mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.lowerAngleRad <= th52
                     && th52 <= mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.upperAngleRad
                   )
            {
                aTh3 = th32;
                aTh4 = th42;
                aTh5 = th52;

                caseMinusPi = true;
            }
        }

        if( !casePlusPi && !caseMinusPi )
        {
            status = false;
        }
    }

    if( status )
    {
        status = checkJointAngles( aTh2, aTh3, aTh4, aTh5 );
    }

    return status;
}


//!************************************************************************
//! Calculate the rank for a set of new joint angles, considering the
//! current values of the joint angles.
//! The cost hierarchy is:
//!     base (max. cost) -> shoulder -> elbow -> wrist (min. cost)
//!
//! => rank=0 means maximum total cost (full movement)
//! => rank=1 means minimum total cost (no movement)
//!
//! @returns the rank of the new joint angles
//!************************************************************************
double RobotArmKinematic::computeJointAnglesRank
    (
    const double aTh2,  //!< angle of base joint [rad]
    const double aTh3,  //!< angle of shoulder joint [rad]
    const double aTh4,  //!< angle of elbow joint [rad]
    const double aTh5   //!< angle of wrist joint [rad]
    ) const
{
    double deltaMaxTh2 = mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_JOINT ).joint.limit.upperAngleRad -
                         mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_JOINT ).joint.limit.lowerAngleRad;
    double deltaMaxTh3 = mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad -
                         mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad;
    double deltaMaxTh4 = mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad -
                         mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad;
    double deltaMaxTh5 = mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.upperAngleRad -
                         mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.lowerAngleRad;

    double diffTh2 = fabs( aTh2 - mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_JOINT ).joint.thetaAngleRad );
    double diffTh3 = fabs( aTh3 - mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.thetaAngleRad );
    double diffTh4 = fabs( aTh4 - mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.thetaAngleRad );
    double diffTh5 = fabs( aTh5 - mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.thetaAngleRad );

    double costTh2 = ( 8.0 / 15.0 ) * ( diffTh2 / deltaMaxTh2 );
    double costTh3 = ( 4.0 / 15.0 ) * ( diffTh3 / deltaMaxTh3 );
    double costTh4 = ( 2.0 / 15.0 ) * ( diffTh4 / deltaMaxTh4 );
    double costTh5 = ( 1.0 / 15.0 ) * ( diffTh5 / deltaMaxTh5 );

    double rank = 1.0 - ( costTh2 + costTh3 + costTh4 + costTh5 );
    return rank;
}


//!************************************************************************
//! Convert an angle from degrees to radians
//!
//! @returns the value in radians
//!************************************************************************
double RobotArmKinematic::convertDegToRad
    (
    const double aValue //!< value
    ) const
{
    return aValue * SerialRobot::PI / 180.0;
}


//!************************************************************************
//! Convert an angle from radians to degrees
//!
//! @returns the value in degrees
//!************************************************************************
double RobotArmKinematic::convertRadToDeg
    (
    const double aValue //!< value
    ) const
{
    return aValue * 180.0 / SerialRobot::PI;
}


//!************************************************************************
//! Finds the index in a joint angles vector whose rank is maximum
//! See description in computeJointAnglesRank().
//!
//! @returns the index in vector where rank is maximum
//!************************************************************************
size_t RobotArmKinematic::findBestRank
    (
    const std::vector<JointAngles>& aJointAnglesVec //!< joint angles vector
    ) const
{
    size_t index = 0;
    size_t vecSize = aJointAnglesVec.size();

    if( vecSize )
    {
        std::vector<double> rankVec;

        for( size_t i = 0; i < vecSize; i++ )
        {
            rankVec.push_back( aJointAnglesVec.at( i ).rank );
        }

        index = std::distance( rankVec.begin(), std::max_element( rankVec.begin(), rankVec.end() ) );
    }

    return index;
}


//!************************************************************************
//! Generate the URDF file corresponding to the current robot
//!
//! @returns nothing
//!************************************************************************
/* slot */ void RobotArmKinematic::handleGenerateUrdf()
{
    mRobotArmInstance = SerialRobot::getInstance();
    mRobotPartsVec = mRobotArmInstance->getRobotPartsVec();

    if( mRobotArmInstance->doSanityCheckRobotParts() )
    {
        UrdfHandler urdfHndl;
        urdfHndl.setRobotPartsVec( mRobotPartsVec );

        if( mRobotPartsVec.empty() )
        {
            QMessageBox::warning( this, "Robot Arm Kinematic",
                                  "There are no robot parts defined."
                                  "\nCannot generate an URDF file.",
                                  QMessageBox::Ok );
        }
        else
        {
            urdfHndl.generateFile();
        }
    }
    else
    {
        QMessageBox::critical( this, "Robot Arm Kinematic",
                               "Current robot parts were not successfully parsed."
                               "\nCannot generate an URDF file.",
                               QMessageBox::Ok );
    }
}


//!************************************************************************
//! Generate the XRDF file corresponding to the current robot
//!
//! @returns nothing
//!************************************************************************
/* slot */ void RobotArmKinematic::handleGenerateXrdf()
{
    mRobotArmInstance = SerialRobot::getInstance();
    mRobotPartsVec = mRobotArmInstance->getRobotPartsVec();

    if( mRobotArmInstance->doSanityCheckRobotParts() )
    {
        XrdfHandler xrdfHndl;
        xrdfHndl.setRobotPartsVec( mRobotPartsVec );

        if( mRobotPartsVec.empty() )
        {
            QMessageBox::warning( this, "Robot Arm Kinematic",
                                  "There are no robot parts defined."
                                  "\nCannot generate a XRDF file.",
                                  QMessageBox::Ok );
        }
        else
        {
            xrdfHndl.generateFile();
        }
    }
    else
    {
        QMessageBox::critical( this, "Robot Arm Kinematic",
                               "Current robot parts were not successfully parsed."
                               "\nCannot generate a XRDF file.",
                               QMessageBox::Ok );
    }
}


//!************************************************************************
//! Handle for changing the length of base link
//!
//! @returns nothing
//!************************************************************************
/* slot */ void RobotArmKinematic::handleBaseLengthChanged
    (
    double aValue   //!< link length
    )
{
    if( aValue > 0 )
    {
        mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_LINK ).link.geometry.cylinder.length = aValue;
        mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_LINK ).link.origin.z = 0.5 * aValue;
        mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_COLUMN_LINK ).link.origin.z = aValue;
        mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.origin.z = aValue +
            mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_JOINT ).joint.origin.z;

        if( !SerialRobot::USE_FIXED_BASE_MASS )
        {
            mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_LINK ).link.mass =
                mRobotArmInstance->calculateLinkMass( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_LINK ).link, SerialRobot::AVG_DENSITY );

            mMainUi->BaseMassSpinBox->setValue( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_LINK ).link.mass );
        }

        mRobotArmInstance->calculateInertiaTensor( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_LINK ).link );

        mRobotArmInstance->getRobotPartsVec() = mRobotPartsVec;

        mMainUi->TotalLengthValue->setText( QString::number( mRobotArmInstance->getTotalLength(), 'f', 3 ) );
        updateEndEffectorCoordinates();
    }
}


//!************************************************************************
//! Handle for changing the radius of base link
//!
//! @returns nothing
//!************************************************************************
/* slot */ void RobotArmKinematic::handleBaseRadiusChanged
    (
    double aValue   //!< link radius
    )
{
    if( aValue > 0 )
    {
        mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_LINK ).link.geometry.cylinder.radius = aValue;

        if( !SerialRobot::USE_FIXED_BASE_MASS )
        {
            mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_LINK ).link.mass =
                mRobotArmInstance->calculateLinkMass( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_LINK ).link, SerialRobot::AVG_DENSITY );

            mMainUi->BaseMassSpinBox->setValue( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_LINK ).link.mass );
        }

        mRobotArmInstance->calculateInertiaTensor( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_LINK ).link );

        mRobotArmInstance->getRobotPartsVec() = mRobotPartsVec;
    }
}


//!************************************************************************
//! Handle for changing the mass of base link
//!
//! @returns nothing
//!************************************************************************
/* slot */ void RobotArmKinematic::handleBaseMassChanged
    (
    double aValue   //!< link mass
    )
{
    if( aValue > 0 )
    {
        mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_LINK ).link.mass = aValue;
        mRobotArmInstance->calculateInertiaTensor( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_LINK ).link );

        mRobotArmInstance->getRobotPartsVec() = mRobotPartsVec;
    }
}


//!************************************************************************
//! Handle for changing the length of column link
//!
//! @returns nothing
//!************************************************************************
/* slot */ void RobotArmKinematic::handleColumnLengthChanged
    (
    double aValue   //!< link length
    )
{
    if( aValue > 0 )
    {
        mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_COLUMN_LINK ).link.geometry.cylinder.length = aValue;
        mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_JOINT ).joint.origin.z = 0.5 * aValue;

        mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_COLUMN_LINK ).link.mass =
            mRobotArmInstance->calculateLinkMass( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_COLUMN_LINK ).link, SerialRobot::AVG_DENSITY );
        mRobotArmInstance->calculateInertiaTensor( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_COLUMN_LINK ).link );
        mMainUi->ColumnMassSpinBox->setValue( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_COLUMN_LINK ).link.mass );

        mRobotArmInstance->getRobotPartsVec() = mRobotPartsVec;

        mMainUi->TotalLengthValue->setText( QString::number( mRobotArmInstance->getTotalLength(), 'f', 3 ) );
        updateEndEffectorCoordinates();
    }
}


//!************************************************************************
//! Handle for changing the radius of column link
//!
//! @returns nothing
//!************************************************************************
/* slot */ void RobotArmKinematic::handleColumnRadiusChanged
    (
    double aValue   //!< link radius
    )
{
    if( aValue > 0 )
    {
        mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_COLUMN_LINK ).link.geometry.cylinder.radius = aValue;

        mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_COLUMN_LINK ).link.mass =
            mRobotArmInstance->calculateLinkMass( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_COLUMN_LINK ).link, SerialRobot::AVG_DENSITY );
        mRobotArmInstance->calculateInertiaTensor( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_COLUMN_LINK ).link );
        mMainUi->ColumnMassSpinBox->setValue( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_COLUMN_LINK ).link.mass );

        mRobotArmInstance->getRobotPartsVec() = mRobotPartsVec;
    }
}


//!************************************************************************
//! Handle for changing the mass of column link
//!
//! @returns nothing
//!************************************************************************
/* slot */ void RobotArmKinematic::handleColumnMassChanged
    (
    double aValue   //!< link mass
    )
{
    if( aValue > 0 )
    {
        mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_COLUMN_LINK ).link.mass = aValue;
        mRobotArmInstance->calculateInertiaTensor( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_COLUMN_LINK ).link );

        mRobotArmInstance->getRobotPartsVec() = mRobotPartsVec;
    }
}


//!************************************************************************
//! Handle for changing the length of upper arm link
//!
//! @returns nothing
//!************************************************************************
/* slot */ void RobotArmKinematic::handleUpperArmLengthChanged
    (
    double aValue   //!< link length
    )
{
    if( aValue > 0 )
    {
        mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_UPPER_ARM_LINK ).link.geometry.cylinder.length = aValue;
        mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_UPPER_ARM_LINK ).link.origin.z = 0.5 * aValue;
        mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.origin.z = aValue;

        mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_UPPER_ARM_LINK ).link.mass =
            mRobotArmInstance->calculateLinkMass( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_UPPER_ARM_LINK ).link, SerialRobot::AVG_DENSITY );
        mRobotArmInstance->calculateInertiaTensor( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_UPPER_ARM_LINK ).link );
        mMainUi->UpperArmMassSpinBox->setValue( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_UPPER_ARM_LINK ).link.mass );

        mRobotArmInstance->getRobotPartsVec() = mRobotPartsVec;

        mMainUi->TotalLengthValue->setText( QString::number( mRobotArmInstance->getTotalLength(), 'f', 3 ) );
        updateEndEffectorCoordinates();
    }
}


//!************************************************************************
//! Handle for changing the radius of upper arm link
//!
//! @returns nothing
//!************************************************************************
/* slot */ void RobotArmKinematic::handleUpperArmRadiusChanged
    (
    double aValue   //!< link radius
    )
{
    if( aValue > 0 )
    {
        mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_UPPER_ARM_LINK ).link.geometry.cylinder.radius = aValue;

        mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_UPPER_ARM_LINK ).link.mass =
            mRobotArmInstance->calculateLinkMass( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_UPPER_ARM_LINK ).link, SerialRobot::AVG_DENSITY );
        mRobotArmInstance->calculateInertiaTensor( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_UPPER_ARM_LINK ).link );
        mMainUi->UpperArmMassSpinBox->setValue( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_UPPER_ARM_LINK ).link.mass );

        mRobotArmInstance->getRobotPartsVec() = mRobotPartsVec;
    }
}


//!************************************************************************
//! Handle for changing the mass of upper arm link
//!
//! @returns nothing
//!************************************************************************
/* slot */ void RobotArmKinematic::handleUpperArmMassChanged
    (
    double aValue   //!< link mass
    )
{
    if( aValue > 0 )
    {
        mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_UPPER_ARM_LINK ).link.mass = aValue;
        mRobotArmInstance->calculateInertiaTensor( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_UPPER_ARM_LINK ).link );

        mRobotArmInstance->getRobotPartsVec() = mRobotPartsVec;
    }
}


//!************************************************************************
//! Handle for changing the length of lower arm link
//!
//! @returns nothing
//!************************************************************************
/* slot */ void RobotArmKinematic::handleLowerArmLengthChanged
    (
    double aValue   //!< link length
    )
{
    if( aValue > 0 )
    {
        mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_LOWER_ARM_LINK ).link.geometry.cylinder.length = aValue;
        mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_LOWER_ARM_LINK ).link.origin.z = 0.5 * aValue;
        mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.origin.z = aValue;

        mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_LOWER_ARM_LINK ).link.mass =
            mRobotArmInstance->calculateLinkMass( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_LOWER_ARM_LINK ).link, SerialRobot::AVG_DENSITY );
        mRobotArmInstance->calculateInertiaTensor( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_LOWER_ARM_LINK ).link );
        mMainUi->LowerArmMassSpinBox->setValue( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_LOWER_ARM_LINK ).link.mass );

        mRobotArmInstance->getRobotPartsVec() = mRobotPartsVec;

        mMainUi->TotalLengthValue->setText( QString::number( mRobotArmInstance->getTotalLength(), 'f', 3 ) );
        updateEndEffectorCoordinates();
    }
}


//!************************************************************************
//! Handle for changing the radius of lower arm link
//!
//! @returns nothing
//!************************************************************************
/* slot */ void RobotArmKinematic::handleLowerArmRadiusChanged
    (
    double aValue   //!< link radius
    )
{
    if( aValue > 0 )
    {
        mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_LOWER_ARM_LINK ).link.geometry.cylinder.radius = aValue;

        mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_LOWER_ARM_LINK ).link.mass =
            mRobotArmInstance->calculateLinkMass( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_LOWER_ARM_LINK ).link, SerialRobot::AVG_DENSITY );
        mRobotArmInstance->calculateInertiaTensor( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_LOWER_ARM_LINK ).link );
        mMainUi->LowerArmMassSpinBox->setValue( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_LOWER_ARM_LINK ).link.mass );

        mRobotArmInstance->getRobotPartsVec() = mRobotPartsVec;
    }
}


//!************************************************************************
//! Handle for changing the mass of lower arm link
//!
//! @returns nothing
//!************************************************************************
/* slot */ void RobotArmKinematic::handleLowerArmMassChanged
    (
    double aValue   //!< link mass
    )
{
    if( aValue > 0 )
    {
        mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_LOWER_ARM_LINK ).link.mass = aValue;

        mRobotArmInstance->calculateInertiaTensor( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_LOWER_ARM_LINK ).link );

        mRobotArmInstance->getRobotPartsVec() = mRobotPartsVec;
    }
}


//!************************************************************************
//! Handle for changing the length of end effector link
//!
//! @returns nothing
//!************************************************************************
/* slot */ void RobotArmKinematic::handleEndEffectorLengthChanged
    (
    double aValue   //!< link length
    )
{
    if( aValue > 0 )
    {
        mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_END_EFFECTOR_LINK ).link.geometry.cylinder.length = aValue;
        mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_END_EFFECTOR_LINK ).link.origin.z = 0.5 * aValue;

        mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_END_EFFECTOR_LINK ).link.mass =
            mRobotArmInstance->calculateLinkMass( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_END_EFFECTOR_LINK ).link, SerialRobot::AVG_DENSITY );
        mRobotArmInstance->calculateInertiaTensor( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_END_EFFECTOR_LINK ).link );
        mMainUi->EndEffectorMassSpinBox->setValue( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_END_EFFECTOR_LINK ).link.mass );

        mRobotArmInstance->getRobotPartsVec() = mRobotPartsVec;

        mMainUi->TotalLengthValue->setText( QString::number( mRobotArmInstance->getTotalLength(), 'f', 3 ) );
        updateEndEffectorCoordinates();
    }
}


//!************************************************************************
//! Handle for changing the radius of end effector link
//!
//! @returns nothing
//!************************************************************************
/* slot */ void RobotArmKinematic::handleEndEffectorRadiusChanged
    (
    double aValue   //!< link radius
    )
{
    if( aValue > 0 )
    {
        mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_END_EFFECTOR_LINK ).link.geometry.cylinder.radius = aValue;

        mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_END_EFFECTOR_LINK ).link.mass =
            mRobotArmInstance->calculateLinkMass( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_END_EFFECTOR_LINK ).link, SerialRobot::AVG_DENSITY );
        mRobotArmInstance->calculateInertiaTensor( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_END_EFFECTOR_LINK ).link );
        mMainUi->EndEffectorMassSpinBox->setValue( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_END_EFFECTOR_LINK ).link.mass );

        mRobotArmInstance->getRobotPartsVec() = mRobotPartsVec;
    }
}


//!************************************************************************
//! Handle for changing the mass of end effector link
//!
//! @returns nothing
//!************************************************************************
/* slot */ void RobotArmKinematic::handleEndEffectorMassChanged
    (
    double aValue   //!< link mass
    )
{
    if( aValue > 0 )
    {
        mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_END_EFFECTOR_LINK ).link.mass = aValue;
        mRobotArmInstance->calculateInertiaTensor( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_END_EFFECTOR_LINK ).link );

        mRobotArmInstance->getRobotPartsVec() = mRobotPartsVec;
    }
}


//!************************************************************************
//! Updates related to the kinematic type
//!
//! @returns nothing
//!************************************************************************
/* slot */ void RobotArmKinematic::handleKinematicType
    (
    int aIndex      //!< kinematic type
    )
{
    switch( aIndex )
    {
        case KINEMATIC_TYPE_DIRECT:
            mKinematicType = KINEMATIC_TYPE_DIRECT;

            makeReadonly( mMainUi->JointBaseMinSpinBox, false );
            makeReadonly( mMainUi->JointBaseCrtSpinBox, false, true );
            makeReadonly( mMainUi->JointBaseMaxSpinBox, false );

            makeReadonly( mMainUi->JointShoulderMinSpinBox, false );
            makeReadonly( mMainUi->JointShoulderCrtSpinBox, false, true );
            makeReadonly( mMainUi->JointShoulderMaxSpinBox, false );

            makeReadonly( mMainUi->JointElbowMinSpinBox, false );
            makeReadonly( mMainUi->JointElbowCrtSpinBox, false, true );
            makeReadonly( mMainUi->JointElbowMaxSpinBox, false );

            makeReadonly( mMainUi->JointWristMinSpinBox, false );
            makeReadonly( mMainUi->JointWristCrtSpinBox, false, true );
            makeReadonly( mMainUi->JointWristMaxSpinBox, false );

            makeReadonly( mMainUi->EeXSpinBox, true, true );
            makeReadonly( mMainUi->EeYSpinBox, true, true );
            makeReadonly( mMainUi->EeZSpinBox, true, true );

            mMainUi->StatusIkmLabel->setVisible( false );

            updateEndEffectorCoordinates();
            break;

        case KINEMATIC_TYPE_INVERSE:
            mKinematicType = KINEMATIC_TYPE_INVERSE;

            makeReadonly( mMainUi->JointBaseMinSpinBox, true );
            makeReadonly( mMainUi->JointBaseCrtSpinBox, true, true );
            makeReadonly( mMainUi->JointBaseMaxSpinBox, true );

            makeReadonly( mMainUi->JointShoulderMinSpinBox, true );
            makeReadonly( mMainUi->JointShoulderCrtSpinBox, true, true );
            makeReadonly( mMainUi->JointShoulderMaxSpinBox, true );

            makeReadonly( mMainUi->JointElbowMinSpinBox, true );
            makeReadonly( mMainUi->JointElbowCrtSpinBox, true, true );
            makeReadonly( mMainUi->JointElbowMaxSpinBox, true );

            makeReadonly( mMainUi->JointWristMinSpinBox, true );
            makeReadonly( mMainUi->JointWristCrtSpinBox, true, true );
            makeReadonly( mMainUi->JointWristMaxSpinBox, true );

            makeReadonly( mMainUi->EeXSpinBox, false, true );
            makeReadonly( mMainUi->EeYSpinBox, false, true );
            makeReadonly( mMainUi->EeZSpinBox, false, true );

            mMainUi->StatusIkmLabel->setVisible( true );
            mMainUi->StatusIkmLabel->setText( "" );
            break;

        default:
            break;
    }
}


//!************************************************************************
//! Updates related to the min angle of base joint
//!
//! @returns nothing
//!************************************************************************
/* slot */ void RobotArmKinematic::handleJointBaseAngleMinChanged
    (
    double aValue   //!< min angle [deg]
    )
{
    double valueRad = convertDegToRad( aValue );

    if( valueRad >= SerialRobot::MIN_BASE_JOINT_ANGLE_RAD
     && valueRad < mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_JOINT ).joint.thetaAngleRad
      )
    {
        mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_JOINT ).joint.limit.lowerAngleRad = valueRad;
        mRobotArmInstance->getRobotPartsVec() = mRobotPartsVec;

        mMainUi->JointBaseCrtSpinBox->setMinimum( aValue );
    }
}


//!************************************************************************
//! Updates related to the angle of base joint
//! *** Direct kinematic model ***
//!
//! @returns nothing
//!************************************************************************
/* slot */ void RobotArmKinematic::handleJointBaseAngleCrtChanged
    (
    double aValue   //!< joint angle [deg]
    )
{
    mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_JOINT ).joint.thetaAngleRad = convertDegToRad( aValue );
    mRobotArmInstance->getRobotPartsVec() = mRobotPartsVec;

    if( KINEMATIC_TYPE_DIRECT == mKinematicType )
    {
        mMainUi->JointBaseMinSpinBox->setMaximum( aValue );
        mMainUi->JointBaseMaxSpinBox->setMinimum( aValue );

        updateEndEffectorCoordinates();
    }
}


//!************************************************************************
//! Updates related to the max angle of base joint
//!
//! @returns nothing
//!************************************************************************
/* slot */ void RobotArmKinematic::handleJointBaseAngleMaxChanged
    (
    double aValue   //!< max angle [deg]
    )
{
    double valueRad = convertDegToRad( aValue );

    if( valueRad > mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_JOINT ).joint.thetaAngleRad
     && valueRad <= SerialRobot::MAX_BASE_JOINT_ANGLE_RAD
      )
    {
        mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_JOINT ).joint.limit.upperAngleRad = valueRad;
        mRobotArmInstance->getRobotPartsVec() = mRobotPartsVec;

        mMainUi->JointBaseCrtSpinBox->setMaximum( aValue );
    }
}


//!************************************************************************
//! Updates related to the min angle of shoulder joint
//!
//! @returns nothing
//!************************************************************************
/* slot */ void RobotArmKinematic::handleJointShoulderAngleMinChanged
    (
    double aValue   //!< min angle [deg]
    )
{
    double valueRad = convertDegToRad( aValue );

    if( valueRad >= SerialRobot::MIN_SHOULDER_JOINT_ANGLE_RAD
     && valueRad < mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.thetaAngleRad
      )
    {
        mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.lowerAngleRad = valueRad;
        mRobotArmInstance->getRobotPartsVec() = mRobotPartsVec;

        mMainUi->JointShoulderCrtSpinBox->setMinimum( aValue );
    }
}


//!************************************************************************
//! Updates related to the angle of shoulder joint
//! *** Direct kinematic model ***
//!
//! @returns nothing
//!************************************************************************
/* slot */ void RobotArmKinematic::handleJointShoulderAngleCrtChanged
    (
    double aValue   //!< joint angle [deg]
    )
{
    mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.thetaAngleRad = convertDegToRad( aValue );
    mRobotArmInstance->getRobotPartsVec() = mRobotPartsVec;

    if( KINEMATIC_TYPE_DIRECT == mKinematicType )
    {
        mMainUi->JointShoulderMinSpinBox->setMaximum( aValue );
        mMainUi->JointShoulderMaxSpinBox->setMinimum( aValue );

        updateEndEffectorCoordinates();
    }
}


//!************************************************************************
//! Updates related to the max angle of shoulder joint
//!
//! @returns nothing
//!************************************************************************
/* slot */ void RobotArmKinematic::handleJointShoulderAngleMaxChanged
    (
    double aValue   //!< max angle [deg]
    )
{
    double valueRad = convertDegToRad( aValue );

    if( valueRad > mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.thetaAngleRad
     && valueRad <= SerialRobot::MAX_SHOULDER_JOINT_ANGLE_RAD
      )
    {
        mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.limit.upperAngleRad = valueRad;
        mRobotArmInstance->getRobotPartsVec() = mRobotPartsVec;

        mMainUi->JointShoulderCrtSpinBox->setMaximum( aValue );
    }
}


//!************************************************************************
//! Updates related to the min angle of elbow joint
//!
//! @returns nothing
//!************************************************************************
/* slot */ void RobotArmKinematic::handleJointElbowAngleMinChanged
    (
    double aValue   //!< min angle [deg]
    )
{
    double valueRad = convertDegToRad( aValue );

    if( valueRad >= SerialRobot::MIN_ELBOW_JOINT_ANGLE_RAD
     && valueRad < mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.thetaAngleRad
      )
    {
        mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.lowerAngleRad = valueRad;
        mRobotArmInstance->getRobotPartsVec() = mRobotPartsVec;

        mMainUi->JointElbowCrtSpinBox->setMinimum( aValue );
    }
}


//!************************************************************************
//! Updates related to the angle of elbow joint
//! *** Direct kinematic model ***
//!
//! @returns nothing
//!************************************************************************
/* slot */ void RobotArmKinematic::handleJointElbowAngleCrtChanged
    (
    double aValue   //!< joint angle [deg]
    )
{
    mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.thetaAngleRad = convertDegToRad( aValue );
    mRobotArmInstance->getRobotPartsVec() = mRobotPartsVec;

    if( KINEMATIC_TYPE_DIRECT == mKinematicType )
    {
        mMainUi->JointElbowMinSpinBox->setMaximum( aValue );
        mMainUi->JointElbowMaxSpinBox->setMinimum( aValue );

        updateEndEffectorCoordinates();
    }
}


//!************************************************************************
//! Updates related to the max angle of elbow joint
//!
//! @returns nothing
//!************************************************************************
/* slot */ void RobotArmKinematic::handleJointElbowAngleMaxChanged
    (
    double aValue   //!< max angle [deg]
    )
{
    double valueRad = convertDegToRad( aValue );

    if( valueRad > mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.thetaAngleRad
     && valueRad <= SerialRobot::MAX_ELBOW_JOINT_ANGLE_RAD
      )
    {
        mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.limit.upperAngleRad = valueRad;
        mRobotArmInstance->getRobotPartsVec() = mRobotPartsVec;

        mMainUi->JointElbowCrtSpinBox->setMaximum( aValue );
    }
}


//!************************************************************************
//! Updates related to the min angle of wrist joint
//!
//! @returns nothing
//!************************************************************************
/* slot */ void RobotArmKinematic::handleJointWristAngleMinChanged
    (
    double aValue   //!< min angle [deg]
    )
{
    double valueRad = convertDegToRad( aValue );

    if( valueRad >= SerialRobot::MIN_WRIST_JOINT_ANGLE_RAD
     && valueRad < mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.thetaAngleRad
      )
    {
        mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.lowerAngleRad = valueRad;
        mRobotArmInstance->getRobotPartsVec() = mRobotPartsVec;

        mMainUi->JointWristCrtSpinBox->setMinimum( aValue );
    }
}


//!************************************************************************
//! Updates related to the angle of wrist joint
//! *** Direct kinematic model ***
//!
//! @returns nothing
//!************************************************************************
/* slot */ void RobotArmKinematic::handleJointWristAngleCrtChanged
    (
    double aValue   //!< joint angle [deg]
    )
{
    mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.thetaAngleRad = convertDegToRad( aValue );
    mRobotArmInstance->getRobotPartsVec() = mRobotPartsVec;

    if( KINEMATIC_TYPE_DIRECT == mKinematicType )
    {
        mMainUi->JointWristMinSpinBox->setMaximum( aValue );
        mMainUi->JointWristMaxSpinBox->setMinimum( aValue );

        updateEndEffectorCoordinates();
    }
}


//!************************************************************************
//! Updates related to the max angle of wrist joint
//!
//! @returns nothing
//!************************************************************************
/* slot */ void RobotArmKinematic::handleJointWristAngleMaxChanged
    (
    double aValue   //!< max angle [deg]
    )
{
    double valueRad = convertDegToRad( aValue );

    if( valueRad > mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.thetaAngleRad
     && valueRad <= SerialRobot::MAX_WRIST_JOINT_ANGLE_RAD
      )
    {
        mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.limit.upperAngleRad = valueRad;
        mRobotArmInstance->getRobotPartsVec() = mRobotPartsVec;

        mMainUi->JointWristCrtSpinBox->setMaximum( aValue );
    }
}


//!************************************************************************
//! Updates related to the x coordinate of end-effector
//! *** Inverse kinematic model ***
//!
//! @returns nothing
//!************************************************************************
/* slot */ void RobotArmKinematic::handleEeCoordXChanged
    (
    double aValue   //!< x coordinate [m]
    )
{
    if( KINEMATIC_TYPE_INVERSE == mKinematicType )
    {
        mRobotArmInstance = SerialRobot::getInstance();
        mRobotPartsVec = mRobotArmInstance->getRobotPartsVec();

        SerialRobot::TriaxialPoint endEffectorCoords = mRobotArmInstance->getEndEffectorCoordinates();
        endEffectorCoords.x = aValue;
        mRobotArmInstance->getEndEffectorCoordinates() = endEffectorCoords;

        bool status = computeIkm( endEffectorCoords.x,
                                  endEffectorCoords.y,
                                  endEffectorCoords.z,
                                  mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_LINK ).link.geometry.cylinder.length,
                                  mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_COLUMN_LINK ).link.geometry.cylinder.length,
                                  mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_UPPER_ARM_LINK ).link.geometry.cylinder.length,
                                  mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_LOWER_ARM_LINK ).link.geometry.cylinder.length,
                                  mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_END_EFFECTOR_LINK ).link.geometry.cylinder.length,
                                  mJointAnglesVec
                                 );

        size_t solCount = mJointAnglesVec.size();

        QString statusIkmStr = "Status = ";
        statusIkmStr += status ? "OK" : "failed";
        statusIkmStr += " (";
        statusIkmStr += QString::number( solCount );
        statusIkmStr += ")";

        mMainUi->StatusIkmLabel->setText( statusIkmStr );

        std::cout << "\n For x=" << endEffectorCoords.x << ", "
                            "y=" << endEffectorCoords.y << ", "
                            "z=" << endEffectorCoords.z << ": ";

        if( !solCount )
        {
            std::cout << "no solution.\n";
        }
        else
        {
            std::cout << solCount << " solution(s):\n";

            for( size_t i = 0; i < solCount; i++ )
            {
                std::cout << "th2=" << convertRadToDeg( mJointAnglesVec.at(i).th2 ) << ", "
                          << "th3=" << convertRadToDeg( mJointAnglesVec.at(i).th3 ) << ", "
                          << "th4=" << convertRadToDeg( mJointAnglesVec.at(i).th4 ) << ", "
                          << "th5=" << convertRadToDeg( mJointAnglesVec.at(i).th5 ) << ", "
                          << "rank=" << mJointAnglesVec.at(i).rank << "\n";
            }

            if( solCount > 1 )
            {
                std::cout << "Best solution:\n";
                size_t brIndex = findBestRank( mJointAnglesVec );
                updateJointAnglesFromIkm( mJointAnglesVec.at( brIndex ) );

                std::cout << "th2=" << convertRadToDeg( mJointAnglesVec.at(brIndex).th2 ) << ", "
                          << "th3=" << convertRadToDeg( mJointAnglesVec.at(brIndex).th3 ) << ", "
                          << "th4=" << convertRadToDeg( mJointAnglesVec.at(brIndex).th4 ) << ", "
                          << "th5=" << convertRadToDeg( mJointAnglesVec.at(brIndex).th5 ) << ", "
                          << "rank=" << mJointAnglesVec.at(brIndex).rank << "\n";
            }
        }

        std::cout << std::flush;
    }
}


//!************************************************************************
//! Updates related to the y coordinate of end-effector
//! *** Inverse kinematic model ***
//!
//! @returns nothing
//!************************************************************************
/* slot */ void RobotArmKinematic::handleEeCoordYChanged
    (
    double aValue   //!< y coordinate [m]
    )
{
    if( KINEMATIC_TYPE_INVERSE == mKinematicType )
    {
        mRobotArmInstance = SerialRobot::getInstance();
        mRobotPartsVec = mRobotArmInstance->getRobotPartsVec();

        SerialRobot::TriaxialPoint endEffectorCoords = mRobotArmInstance->getEndEffectorCoordinates();
        endEffectorCoords.y = aValue;
        mRobotArmInstance->getEndEffectorCoordinates() = endEffectorCoords;

        bool status = computeIkm( endEffectorCoords.x,
                                  endEffectorCoords.y,
                                  endEffectorCoords.z,
                                  mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_LINK ).link.geometry.cylinder.length,
                                  mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_COLUMN_LINK ).link.geometry.cylinder.length,
                                  mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_UPPER_ARM_LINK ).link.geometry.cylinder.length,
                                  mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_LOWER_ARM_LINK ).link.geometry.cylinder.length,
                                  mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_END_EFFECTOR_LINK ).link.geometry.cylinder.length,
                                  mJointAnglesVec
                                 );

        size_t solCount = mJointAnglesVec.size();

        QString statusIkmStr = "Status = ";
        statusIkmStr += status ? "OK" : "failed";
        statusIkmStr += " (";
        statusIkmStr += QString::number( solCount );
        statusIkmStr += ")";

        mMainUi->StatusIkmLabel->setText( statusIkmStr );

        std::cout << "\n For x=" << endEffectorCoords.x << ", "
                            "y=" << endEffectorCoords.y << ", "
                            "z=" << endEffectorCoords.z << ": ";

        if( !solCount )
        {
            std::cout << "no solution.\n";
        }
        else
        {
            std::cout << solCount << " solution(s):\n";

            for( size_t i = 0; i < solCount; i++ )
            {
                std::cout << "th2=" << convertRadToDeg( mJointAnglesVec.at(i).th2 ) << ", "
                          << "th3=" << convertRadToDeg( mJointAnglesVec.at(i).th3 ) << ", "
                          << "th4=" << convertRadToDeg( mJointAnglesVec.at(i).th4 ) << ", "
                          << "th5=" << convertRadToDeg( mJointAnglesVec.at(i).th5 ) << ", "
                          << "rank=" << mJointAnglesVec.at(i).rank << "\n";

            }

            if( solCount > 1 )
            {
                std::cout << "Best solution:\n";
                size_t brIndex = findBestRank( mJointAnglesVec );
                updateJointAnglesFromIkm( mJointAnglesVec.at( brIndex ) );

                std::cout << "th2=" << convertRadToDeg( mJointAnglesVec.at(brIndex).th2 ) << ", "
                          << "th3=" << convertRadToDeg( mJointAnglesVec.at(brIndex).th3 ) << ", "
                          << "th4=" << convertRadToDeg( mJointAnglesVec.at(brIndex).th4 ) << ", "
                          << "th5=" << convertRadToDeg( mJointAnglesVec.at(brIndex).th5 ) << ", "
                          << "rank=" << mJointAnglesVec.at(brIndex).rank << "\n";
            }
        }

        std::cout << std::flush;
    }
}


//!************************************************************************
//! Updates related to the z coordinate of end-effector
//! *** Inverse kinematic model ***
//!
//! @returns nothing
//!************************************************************************
/* slot */ void RobotArmKinematic::handleEeCoordZChanged
    (
    double aValue   //!< z coordinate [m]
    )
{
    if( KINEMATIC_TYPE_INVERSE == mKinematicType )
    {
        mRobotArmInstance = SerialRobot::getInstance();
        mRobotPartsVec = mRobotArmInstance->getRobotPartsVec();

        SerialRobot::TriaxialPoint endEffectorCoords = mRobotArmInstance->getEndEffectorCoordinates();
        endEffectorCoords.z = aValue;
        mRobotArmInstance->getEndEffectorCoordinates() = endEffectorCoords;

        bool status = computeIkm( endEffectorCoords.x,
                                  endEffectorCoords.y,
                                  endEffectorCoords.z,
                                  mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_LINK ).link.geometry.cylinder.length,
                                  mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_COLUMN_LINK ).link.geometry.cylinder.length,
                                  mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_UPPER_ARM_LINK ).link.geometry.cylinder.length,
                                  mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_LOWER_ARM_LINK ).link.geometry.cylinder.length,
                                  mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_END_EFFECTOR_LINK ).link.geometry.cylinder.length,
                                  mJointAnglesVec
                                 );

        size_t solCount = mJointAnglesVec.size();

        QString statusIkmStr = "Status = ";
        statusIkmStr += status ? "OK" : "failed";
        statusIkmStr += " (";
        statusIkmStr += QString::number( solCount );
        statusIkmStr += ")";

        mMainUi->StatusIkmLabel->setText( statusIkmStr );

        std::cout << "\n For x=" << endEffectorCoords.x << ", "
                            "y=" << endEffectorCoords.y << ", "
                            "z=" << endEffectorCoords.z << ": ";

        if( !solCount )
        {
            std::cout << "no solution.\n";
        }
        else
        {
            std::cout << solCount << " solution(s):\n";

            for( size_t i = 0; i < solCount; i++ )
            {
                std::cout << "th2=" << convertRadToDeg( mJointAnglesVec.at(i).th2 ) << ", "
                          << "th3=" << convertRadToDeg( mJointAnglesVec.at(i).th3 ) << ", "
                          << "th4=" << convertRadToDeg( mJointAnglesVec.at(i).th4 ) << ", "
                          << "th5=" << convertRadToDeg( mJointAnglesVec.at(i).th5 ) << ", "
                          << "rank=" << mJointAnglesVec.at(i).rank << "\n";

            }

            if( solCount > 1 )
            {
                std::cout << "Best solution:\n";
                size_t brIndex = findBestRank( mJointAnglesVec );
                updateJointAnglesFromIkm( mJointAnglesVec.at( brIndex ) );

                std::cout << "th2=" << convertRadToDeg( mJointAnglesVec.at(brIndex).th2 ) << ", "
                          << "th3=" << convertRadToDeg( mJointAnglesVec.at(brIndex).th3 ) << ", "
                          << "th4=" << convertRadToDeg( mJointAnglesVec.at(brIndex).th4 ) << ", "
                          << "th5=" << convertRadToDeg( mJointAnglesVec.at(brIndex).th5 ) << ", "
                          << "rank=" << mJointAnglesVec.at(brIndex).rank << "\n";
            }
        }

        std::cout << std::flush;
    }
}


//!************************************************************************
//! Set the readonly attributes for a spinbox by status
//!
//! @returns nothing
//!************************************************************************
void RobotArmKinematic::makeReadonly
    (
    QDoubleSpinBox*&    aSpinBox,           //!< spinbox
    bool                aStatus,            //!< true for readonly, false otherwise
    bool                aUseCustomColors    //!< true for customized colors
    )
{
    aSpinBox->setReadOnly( aStatus );

    if( aUseCustomColors )
    {
        QFont font = aSpinBox->font();
        font.setBold( aStatus );
        aSpinBox->setFont( font );

        if( aStatus )
        {
            aSpinBox->setStyleSheet( "color: rgb(143, 240, 164); background-color: rgb(0, 0, 0)" );
        }
        else
        {
            aSpinBox->setStyleSheet( "color: rgb(0, 0, 0); background-color: rgb(255, 255, 255)" );
        }
    }
}


//!************************************************************************
//! Refine an angle to belong in [-PI..PI]
//!
//! @returns nothing
//!************************************************************************
void RobotArmKinematic::refineAnglePlusMinusPi
    (
    double& aValue      //!< angle [rad]
    ) const
{
    if( aValue < -SerialRobot::PI )
    {
        aValue += 2 * SerialRobot::PI;
    }
    else if( aValue > SerialRobot::PI )
    {
        aValue -= 2 * SerialRobot::PI;
    }
}


//!************************************************************************
//! Refine two angles which are both close to -PI/2.
//!
//! @returns nothing
//!************************************************************************
void RobotArmKinematic::refineAnglesMinusPi
    (
    double& aTh4,       //!< angle of elbow joint [rad]
    double& aTh5        //!< angle of wrist joint [rad]
    ) const
{
    if( fabs( aTh4 + 0.5 * SerialRobot::PI ) <= ZERO_RAD_THD
     && fabs( aTh5 + 0.5 * SerialRobot::PI ) <= ZERO_RAD_THD
      )
    {
        aTh4 = -0.5 * SerialRobot::PI;
        aTh5 = -0.5 * SerialRobot::PI;
    }
}


//!************************************************************************
//! Refine two angles which are both close to +PI/2.
//!
//! @returns nothing
//!************************************************************************
void RobotArmKinematic::refineAnglesPlusPi
    (
    double& aTh4,       //!< angle of elbow joint [rad]
    double& aTh5        //!< angle of wrist joint [rad]
    ) const
{
    if( fabs( aTh4 - 0.5 * SerialRobot::PI ) <= ZERO_RAD_THD
     && fabs( aTh5 - 0.5 * SerialRobot::PI ) <= ZERO_RAD_THD
      )
    {
        aTh4 = 0.5 * SerialRobot::PI;
        aTh5 = 0.5 * SerialRobot::PI;
    }
}


//!************************************************************************
//! Refine the angle of base joint, based on tan() periodicity and limits.
//! Note: th2=0 is between 3rd and 4th quadrants.
//!
//! @returns nothing
//!************************************************************************
void RobotArmKinematic::refineTh2Angle
    (
    double& aTh2        //!< angle of base joint [rad]
    ) const
{
    double min = mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_JOINT ).joint.limit.lowerAngleRad;
    double max = mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_JOINT ).joint.limit.upperAngleRad;

    if( aTh2 > max
     && aTh2 - SerialRobot::PI >= min )
    {
        aTh2 -= SerialRobot::PI;
    }
    else if( aTh2 < min
          && aTh2 + SerialRobot::PI <= max )
    {
        aTh2 += SerialRobot::PI;
    }
}


//!************************************************************************
//! Update the end-effector coordinates
//!
//! @returns nothing
//!************************************************************************
void RobotArmKinematic::updateEndEffectorCoordinates()
{
    SerialRobot::TriaxialPoint endEffectorCoords = mRobotArmInstance->calculateEndEffectorCoordinates();

    mMainUi->EeXSpinBox->setValue( endEffectorCoords.x );
    mMainUi->EeYSpinBox->setValue( endEffectorCoords.y );
    mMainUi->EeZSpinBox->setValue( endEffectorCoords.z );
}


//!************************************************************************
//! Update the joint angles after solving the inverse kinematic model
//!
//! @returns nothing
//!************************************************************************
void RobotArmKinematic::updateJointAnglesFromIkm
    (
    const JointAngles aJointAngles  //!< joint angles
    )
{
    mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_JOINT ).joint.thetaAngleRad = aJointAngles.th2;
    mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.thetaAngleRad = aJointAngles.th3;
    mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.thetaAngleRad = aJointAngles.th4;
    mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.thetaAngleRad = aJointAngles.th5;

    mMainUi->JointBaseCrtSpinBox->setValue( convertRadToDeg( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_JOINT ).joint.thetaAngleRad ) );
    mMainUi->JointShoulderCrtSpinBox->setValue( convertRadToDeg( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_SHOULDER_JOINT ).joint.thetaAngleRad ) );
    mMainUi->JointElbowCrtSpinBox->setValue( convertRadToDeg( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_ELBOW_JOINT ).joint.thetaAngleRad ) );
    mMainUi->JointWristCrtSpinBox->setValue( convertRadToDeg( mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_WRIST_JOINT ).joint.thetaAngleRad ) );
}


//!************************************************************************
//! Validate a set of joint angles for a target end-effector position
//!
//! @returns true if the set of joint angles is valid
//!************************************************************************
bool RobotArmKinematic::validateIkmSet
    (
    const double aTh2,  //!< angle of base joint [rad]
    const double aTh3,  //!< angle of shoulder joint [rad]
    const double aTh4,  //!< angle of elbow joint [rad]
    const double aTh5,  //!< angle of wrist joint [rad]
    const double aX,    //!< target end-effector x coordinate
    const double aY,    //!< target end-effector y coordinate
    const double aZ     //!< target end-effector z coordinate
    ) const
{
    bool status = false;

    double l1 = mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_BASE_LINK ).link.geometry.cylinder.length;
    double l2 = mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_COLUMN_LINK ).link.geometry.cylinder.length;
    double l3 = mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_UPPER_ARM_LINK ).link.geometry.cylinder.length;
    double l4 = mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_LOWER_ARM_LINK ).link.geometry.cylinder.length;
    double l5 = mRobotPartsVec.at( SerialRobot::PARTS_CHAIN_END_EFFECTOR_LINK ).link.geometry.cylinder.length;

    double m = l3 * cos( aTh3 ) + l4 * cos( aTh3 + aTh4 ) + l5 * cos( aTh3 + aTh4 + aTh5 );
    double n = l3 * sin( aTh3 ) + l4 * sin( aTh3 + aTh4 ) + l5 * sin( aTh3 + aTh4 + aTh5 );

    double calcX = cos( aTh2 ) * m;
    double calcY = sin( aTh2 ) * m;
    double calcZ = l1 + l2 + n;

    status = ( fabs( aX - calcX ) < ZERO_L_THD )
          && ( fabs( aY - calcY ) < ZERO_L_THD )
          && ( fabs( aZ - calcZ ) < ZERO_L_THD );

    return status;
}
