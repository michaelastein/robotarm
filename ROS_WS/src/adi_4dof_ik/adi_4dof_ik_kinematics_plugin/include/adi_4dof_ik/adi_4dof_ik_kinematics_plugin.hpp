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
adi_4dof_ik_kinematics_plugin.h

This file contains the definitions for the 4DOF robot inverse kinematic plugin.
*/

#ifndef adi_4dof_ik_kinematics_plugin_h
#define adi_4dof_ik_kinematics_plugin_h

#include "SerialRobot.hpp"

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>

namespace adi_4dof_ik_kinematics_plugin
{

//************************************************************************
// Class for handling 4DOF robot arm inverse kinematic plugin
//************************************************************************
class ADI_4DOF_KinematicsPlugin : public kinematics::KinematicsBase
{
    //************************************************************************
    // constants and types
    //************************************************************************
    public:
        typedef struct
        {
            double th2;     //!< angle of base joint [rad]
            double th3;     //!< angle of shoulder joint [rad]
            double th4;     //!< angle of elbow joint [rad]
            double th5;     //!< angle of wrist joint [rad]
            double rank;    //!< rank (1-max, 0-min)
        }JointAngles;

    private:
        static const int NR_JOINTS = 4; // robot arm is 4DOF (4R)

        static constexpr double ZERO_L_THD = 1.e-2;                             // 1 cm
        static constexpr double ZERO_RAD_THD = 0.01 * SerialRobot::PI / 180.0;  // 1.74533e-4 rad


    //************************************************************************
    // functions
    //************************************************************************
    public:
        ADI_4DOF_KinematicsPlugin();

        ~ADI_4DOF_KinematicsPlugin();


        const std::vector<std::string>& getJointNames() const;

        const std::vector<std::string>& getLinkNames() const;


        bool getPositionFK
            (
            const std::vector<std::string>&         aLinkNames,     //!< set of links for which FK needs to be computed
            const std::vector<double>&              aJointAngles,   //!< joint angles for which FK needs to be computed
            std::vector<geometry_msgs::msg::Pose>&  aPoses          //!< the resultant set of poses
            ) const override;

        bool getPositionIK
            (
             const geometry_msgs::msg::Pose&            aIkPose,        //!< target end-effector pose
             const std::vector<double>&                 aIkSeedState,   //!< seed (not used)
             std::vector<double>&                       aSolution,      //!< solution vector
             moveit_msgs::msg::MoveItErrorCodes&        aErrorCode,     //!< error code
             const kinematics::KinematicsQueryOptions&  aOptions = kinematics::KinematicsQueryOptions()  //!< options (not used)
             ) const override;


        bool initialize
            (
            const rclcpp::Node::SharedPtr&  aNode,                  //!< node
            const moveit::core::RobotModel& aRobotModel,            //!< robot model
            const std::string&              aGroupName,             //!< group name
            const std::string&              aBaseFrame,             //!< base frame
            const std::vector<std::string>& aTipFrames,             //!< tip frames
            double                          aSearchDiscretization   //!< search discretization
            ) override;


        bool searchPositionIK
            (
            const geometry_msgs::msg::Pose&             aIkPose,        //!< target end-effector pose
            const std::vector<double>&                  aIkSeedState,   //!< seed (not used)
            double                                      aTimeout,       //!< timeout (not used)
            std::vector<double>&                        aSolution,      //!< solution vector
            moveit_msgs::msg::MoveItErrorCodes&         aErrorCode,     //!< error code
            const kinematics::KinematicsQueryOptions&   aOptions = kinematics::KinematicsQueryOptions()  //!< options (not used)
            ) const override;

        bool searchPositionIK
            (
            const geometry_msgs::msg::Pose&             aIkPose,            //!< target end-effector pose
            const std::vector<double>&                  aIkSeedState,       //!< seed (not used)
            double                                      aTimeout,           //!< timeout (not used)
            const std::vector<double>&                  aConsistencyLimits, //!< consistency limits (not used)
            std::vector<double>&                        aSolution,          //!< solution vector
            moveit_msgs::msg::MoveItErrorCodes&         aErrorCode,         //!< error code
            const kinematics::KinematicsQueryOptions&   aOptions = kinematics::KinematicsQueryOptions()  //!< options (not used)
            ) const override;

        bool searchPositionIK
            (
            const geometry_msgs::msg::Pose&             aIkPose,            //!< target end-effector pose
            const std::vector<double>&                  aIkSeedState,       //!< seed (not used)
            double                                      aTimeout,           //!< timeout (not used)
            const std::vector<double>&                  aConsistencyLimits, //!< consistency limits (not used)
            std::vector<double>&                        aSolution,          //!< solution vector
            const IKCallbackFn&                         aSolutionCallback,  //!< callback function (not used)
            moveit_msgs::msg::MoveItErrorCodes&         aErrorCode,         //!< error code
            const kinematics::KinematicsQueryOptions&   aOptions = kinematics::KinematicsQueryOptions()  //!< options (not used)
            ) const override;


        bool searchPositionIK
            (
            const geometry_msgs::msg::Pose&             aIkPose,            //!< target end-effector pose
            const std::vector<double>&                  aIkSeedState,       //!< seed (not used)
            double                                      aTimeout,           //!< timeout (not used)
            std::vector<double>&                        aSolution,          //!< solution vector
            const IKCallbackFn&                         aSolutionCallback,  //!< callback function (not used)
            moveit_msgs::msg::MoveItErrorCodes&         aErrorCode,         //!< error code
            const kinematics::KinematicsQueryOptions&   aOptions = kinematics::KinematicsQueryOptions()  //!< options (not used)
            ) const;

        bool searchPositionIK
            (
            const geometry_msgs::msg::Pose&             aIkPose,            //!< target end-effector pose
            const std::vector<double>&                  aIkSeedState,       //!< seed (not used)
            double                                      aTimeout,           //!< timeout (not used)
            std::vector<double>&                        aSolution,          //!< solution vector
            const IKCallbackFn&                         aSolutionCallback,  //!< callback function (not used)
            moveit_msgs::msg::MoveItErrorCodes&         aErrorCode,         //!< error code
            const std::vector<double>&                  aConsistencyLimits, //!< consistency limits (not used)
            const kinematics::KinematicsQueryOptions&   aOptions            //!< options (not used)
            ) const;


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
            ) const;

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

        void updateJointAngles
            (
            const JointAngles aJointAngles  //!< joint angles
            ) const;

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


        int getKDLSegmentIndex
            (
            const std::string&  aName   //!< name
            ) const;


    //************************************************************************
    // variables
    //************************************************************************
    private:
        SerialRobot*                        mRobotArmInstance;  //!< robot arm instance
        std::vector<SerialRobot::RobotPart> mRobotPartsVec;     //!< robot parts vector

        bool                                mIsActive;          //!< true if active
        uint8_t                             mNumJoints;         //!< number of joints

        std::vector<std::string>            mJointNames;        //!< joint names
        std::vector<std::string>            mLinkNames;         //!< link names

        KDL::Chain                          mChain;             //!< kinematic interconnection structure
};

} // namespace

#endif // adi_4dof_ik_kinematics_plugin_h
