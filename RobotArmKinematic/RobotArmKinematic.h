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
RobotArmKinematic.h

This file contains the definitions for the robot arm kinematic.
*/

#ifndef RobotArmKinematic_h
#define RobotArmKinematic_h

#include "SerialRobot.h"

#include <QDoubleSpinBox>
#include <QMainWindow>

#include <vector>


QT_BEGIN_NAMESPACE
namespace Ui
{
    class RobotArmKinematic;
}
QT_END_NAMESPACE


//************************************************************************
// Class for handling 4DOF robot arm kinematics
//************************************************************************
class RobotArmKinematic : public QMainWindow
{
    Q_OBJECT

    //************************************************************************
    // constants and types
    //************************************************************************
    private:
        typedef enum
        {
            KINEMATIC_TYPE_DIRECT,
            KINEMATIC_TYPE_INVERSE
        }KinematicType;

        typedef struct
        {
            double th2;     //!< angle of base joint [rad]
            double th3;     //!< angle of shoulder joint [rad]
            double th4;     //!< angle of elbow joint [rad]
            double th5;     //!< angle of wrist joint [rad]
            double rank;    //!< rank (1-max, 0-min)
        }JointAngles;

        static constexpr double ZERO_L_THD = 1.e-2;                             // 1 cm
        static constexpr double ZERO_RAD_THD = 0.01 * SerialRobot::PI / 180.0;  // 1.74533e-4 rad


    //************************************************************************
    // functions
    //************************************************************************
    public:
        RobotArmKinematic
            (
            QWidget* aParent = nullptr //!< parent widget
            );

        ~RobotArmKinematic();

    private:
        bool checkJointAngles
            (
            double& aTh2,   //!< angle of base joint [rad]
            double& aTh3,   //!< angle of shoulder joint [rad]
            double& aTh4,   //!< angle of elbow joint [rad]
            double& aTh5    //!< angle of wrist joint [rad]
            ) const;

        bool computeIkm
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
            );

        bool computeIkmHee
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
            ) const;

        bool computeIkmTh5Eq0
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
            ) const;

        bool computeIkmTh5EqMth4
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
            ) const;

        bool computeIkmTh5EqMth3
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
            ) const;

        bool computeIkmTh4EqMth3
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
            ) const;

        bool computeIkmTh5Th4EqPi2
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
            ) const;

        bool computeIkmTh5Th4EqMpi2
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
            ) const;

        bool computeIkmTh5Th4EqPi
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
            ) const;

        double computeJointAnglesRank
            (
            const double aTh2,  //!< angle of base joint [rad]
            const double aTh3,  //!< angle of shoulder joint [rad]
            const double aTh4,  //!< angle of elbow joint [rad]
            const double aTh5   //!< angle of wrist joint [rad]
            ) const;

        double convertDegToRad
            (
            const double aValue //!< value
            ) const;

        double convertRadToDeg
            (
            const double aValue //!< value
            ) const;

        size_t findBestRank
            (
            const std::vector<JointAngles>& aJointAnglesVec //!< joint angles vector
            ) const;

        void makeReadonly
            (
            QDoubleSpinBox*&    aSpinBox,                   //!< spinbox
            bool                aStatus,                    //!< true for readonly, false otherwise
            bool                aUseCustomColors = false    //!< true for customized colors
            );

        void refineAnglePlusMinusPi
            (
            double& aValue      //!< angle [rad]
            ) const;

        void refineAnglesMinusPi
            (
            double& aTh4,       //!< angle of elbow joint [rad]
            double& aTh5        //!< angle of wrist joint [rad]
            ) const;

        void refineAnglesPlusPi
            (
            double& aTh4,       //!< angle of elbow joint [rad]
            double& aTh5        //!< angle of wrist joint [rad]
            ) const;

        void refineTh2Angle
            (
            double& aTh2        //!< angle of base joint [rad]
            ) const;

        void updateEndEffectorCoordinates();

        void updateJointAnglesFromIkm
            (
            const JointAngles aJointAngles  //!< joint angles
            );

        bool validateIkmSet
            (
            const double aTh2,  //!< angle of base joint [rad]
            const double aTh3,  //!< angle of shoulder joint [rad]
            const double aTh4,  //!< angle of elbow joint [rad]
            const double aTh5,  //!< angle of wrist joint [rad]
            const double aX,    //!< target end-effector x coordinate
            const double aY,    //!< target end-effector y coordinate
            const double aZ     //!< target end-effector z coordinate
            ) const;

    private slots:
        void handleGenerateUrdf();

        void handleGenerateXrdf();


        void handleBaseLengthChanged
            (
            double aValue   //!< link length
            );
        void handleBaseRadiusChanged
            (
            double aValue   //!< link radius
            );
        void handleBaseMassChanged
            (
            double aValue   //!< link mass
            );


        void handleColumnLengthChanged
            (
            double aValue   //!< link length
            );
        void handleColumnRadiusChanged
            (
            double aValue   //!< link radius
            );
        void handleColumnMassChanged
            (
            double aValue   //!< link mass
            );


        void handleUpperArmLengthChanged
            (
            double aValue   //!< link length
            );
        void handleUpperArmRadiusChanged
            (
            double aValue   //!< link radius
            );
        void handleUpperArmMassChanged
            (
            double aValue   //!< link mass
            );


        void handleLowerArmLengthChanged
            (
            double aValue   //!< link length
            );
        void handleLowerArmRadiusChanged
            (
            double aValue   //!< link radius
            );
        void handleLowerArmMassChanged
            (
            double aValue   //!< link mass
            );


        void handleEndEffectorLengthChanged
            (
            double aValue   //!< link length
            );
        void handleEndEffectorRadiusChanged
            (
            double aValue   //!< link radius
            );
        void handleEndEffectorMassChanged
            (
            double aValue   //!< link mass
            );


        void handleJointBaseAngleMinChanged
            (
            double aValue   //!< min angle [deg]
            );
        void handleJointBaseAngleCrtChanged
            (
            double aValue   //!< joint angle [deg]
            );
        void handleJointBaseAngleMaxChanged
            (
            double aValue   //!< max angle [deg]
            );

        void handleJointShoulderAngleMinChanged
            (
            double aValue   //!< min angle [deg]
            );
        void handleJointShoulderAngleCrtChanged
            (
            double aValue   //!< joint angle [deg]
            );
        void handleJointShoulderAngleMaxChanged
            (
            double aValue   //!< max angle [deg]
            );

        void handleJointElbowAngleMinChanged
            (
            double aValue   //!< min angle [deg]
            );
        void handleJointElbowAngleCrtChanged
            (
            double aValue   //!< joint angle [deg]
            );
        void handleJointElbowAngleMaxChanged
            (
            double aValue   //!< max angle [deg]
            );

        void handleJointWristAngleMinChanged
            (
            double aValue   //!< min angle [deg]
            );
        void handleJointWristAngleCrtChanged
            (
            double aValue   //!< joint angle [deg]
            );
        void handleJointWristAngleMaxChanged
            (
            double aValue   //!< max angle [deg]
            );


        void handleEeCoordXChanged
            (
            double aValue   //!< x coordinate [m]
            );
        void handleEeCoordYChanged
            (
            double aValue   //!< y coordinate [m]
            );
        void handleEeCoordZChanged
            (
            double aValue   //!< z coordinate [m]
            );


        void handleKinematicType
            (
            int aIndex      //!< kinematic type
            );

    //************************************************************************
    // variables
    //************************************************************************
    private:
        Ui::RobotArmKinematic*                  mMainUi;            //!< main UI

        SerialRobot*                            mRobotArmInstance;  //!< robot arm instance
        std::vector<SerialRobot::RobotPart>     mRobotPartsVec;     //!< robot parts vector

        KinematicType                           mKinematicType;     //!< kinematic type

        std::vector<JointAngles>                mJointAnglesVec;    //!< vector with joint angles
};

#endif // RobotArmKinematic_h
