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
SerialRobot.h

This file contains the definitions for the serial robot.
*/

#ifndef SerialRobot_h
#define SerialRobot_h

#include <cmath>
#include <cstdint>
#include <map>
#include <string>
#include <vector>


//************************************************************************
// Class for handling serial robots - 4DOF
//************************************************************************
class SerialRobot
{
    //************************************************************************
    // constants and types
    //************************************************************************
    public:
        static constexpr double PI = 4.0 * atan( 1.0 );

        static constexpr double AVG_DENSITY = 1000.0;   // kg/m3

        // The base link is fixed on the surface of the scene, its mass can be constant and large
        // compared to the sum of masses for all moving parts
        static const bool USE_FIXED_BASE_MASS = true;

        // base joint angle limits <=> equivalent to a 90 degs H-FOV
        static constexpr double MIN_BASE_JOINT_ANGLE_RAD = -0.25 * PI;
        static constexpr double MAX_BASE_JOINT_ANGLE_RAD =  0.25 * PI;

        // shoulder joint angle limits
        static constexpr double MIN_SHOULDER_JOINT_ANGLE_RAD = -0.5 * PI;
        static constexpr double MAX_SHOULDER_JOINT_ANGLE_RAD =  0.5 * PI;

        // elbow joint angle limits
        static constexpr double MIN_ELBOW_JOINT_ANGLE_RAD = -0.5 * PI;
        static constexpr double MAX_ELBOW_JOINT_ANGLE_RAD =  0.5 * PI;

        // wrist joint angle limits
        static constexpr double MIN_WRIST_JOINT_ANGLE_RAD = -0.5 * PI;
        static constexpr double MAX_WRIST_JOINT_ANGLE_RAD =  0.5 * PI;


        ///////////////////////////
        /// Constructive parts
        ///////////////////////////
        typedef enum : uint8_t
        {
            PART_UNKNOWN,
            PART_LINK,
            PART_JOINT
        }Part;

        typedef enum : uint8_t
        {
            PARTS_CHAIN_BASE_LINK,
            PARTS_CHAIN_BASE_JOINT,
            PARTS_CHAIN_COLUMN_LINK,
            PARTS_CHAIN_SHOULDER_JOINT,
            PARTS_CHAIN_UPPER_ARM_LINK,
            PARTS_CHAIN_ELBOW_JOINT,
            PARTS_CHAIN_LOWER_ARM_LINK,
            PARTS_CHAIN_WRIST_JOINT,
            PARTS_CHAIN_END_EFFECTOR_LINK,

            // keep this last
            PARTS_CHAIN_COUNT
        }PartsChain;

        static const std::map<PartsChain, std::string> PARTS_CHAIN_NAMES;

        typedef enum : uint8_t
        {
            JOINT_TYPE_UNKNOWN,
            JOINT_TYPE_CONTINUOUS,
            JOINT_TYPE_FIXED,
            JOINT_TYPE_PRISMATIC,
            JOINT_TYPE_REVOLUTE
        }JointType;


        ///////////////////////////
        /// 3D geometric shapes
        ///////////////////////////
        typedef enum : uint8_t
        {
            GEOMETRIC_SHAPE_UNKNOWN,
            GEOMETRIC_SHAPE_CYLINDER,
            GEOMETRIC_SHAPE_PARALLELEPIPED
        }GeometricShape;

        typedef struct
        {
            float length;
            float radius;
        }Cylinder;

        typedef struct
        {
            float xSize;
            float ySize;
            float zSize;
        }Parallelepiped;

        typedef struct
        {
            Cylinder        cylinder;
            Parallelepiped  box;
        }Geometry;


        ///////////////////////////
        /// Axes
        ///////////////////////////
        typedef struct
        {
            int8_t xAxis;
            int8_t yAxis;
            int8_t zAxis;
        }TriaxialOrientation;

        typedef struct
        {
            double x;
            double y;
            double z;
        }TriaxialPoint;


        ///////////////////////////
        /// Joint limits
        ///////////////////////////
        typedef struct
        {
            float effort;
            float lowerAngleRad;
            float upperAngleRad;
            float velocity;
        }JointLimit;


        ///////////////////////////
        /// Materials + Colors
        ///////////////////////////
        typedef enum : uint8_t
        {
            MATERIAL_UNKNOWN,
            MATERIAL_BLUE,
            MATERIAL_BLACK
        }Material;

        typedef struct
        {
            float red;
            float green;
            float blue;
            float alpha;
        }ColorRgba;

        static const ColorRgba COLOR_BLUE;
        static const ColorRgba COLOR_BLACK;

        static constexpr double ANTI_COLLISION_DIST = 0.01;     // 1 cm

        typedef double InertiaTensor[3][3];


        ///////////////////////////
        /// Link
        ///////////////////////////
        typedef struct
        {
            std::string         name;
            GeometricShape      geometricShape;
            Geometry            geometry;
            TriaxialPoint       origin;
            Material            material;
            float               mass;
            InertiaTensor       inertiaTensor;
        }Link;


        ///////////////////////////
        /// Joint
        ///////////////////////////
        typedef struct
        {
            std::string         name;
            JointType           type;
            Link                parentLink;
            Link                childLink;
            TriaxialOrientation orientation;
            TriaxialPoint       origin;
            JointLimit          limit;
            float               defaultPosition;
            float               accelerationLimit;
            float               jerkLimit;
            double              thetaAngleRad;
        }Joint;


        ///////////////////////////
        /// Robot part
        ///////////////////////////
        typedef struct
        {
            Part    partType;
            Link    link;
            Joint   joint;
        }RobotPart;


    //************************************************************************
    // functions
    //************************************************************************
    public:
        SerialRobot();

        static SerialRobot* getInstance();

        TriaxialPoint calculateEndEffectorCoordinates();

        void calculateInertiaTensor
            (
            Link& aLink             //!< link
            );

        double calculateLinkMass
            (
            const Link      aLink,      //!< serial robot link
            const double    aAvgDensity //!< average density [kg/m3]
            );

        double getTotalLength();

        void clearRobotPart
            (
            RobotPart& aRobotPart   //!< robot part
            );

        bool doSanityCheckRobotParts();

        TriaxialPoint& getEndEffectorCoordinates();

        std::vector<RobotPart>& getRobotPartsVec();

        void initializeRobotParts();


    //************************************************************************
    // variables
    //************************************************************************
    private:
        static SerialRobot*         sInstance;      //!< singleton
        std::vector<RobotPart>      mRobotPartsVec; //!< robot parts vector, starting from base to end-effector
        TriaxialPoint               mEndEffector;   //!< coordinates of the end-effector
        double                      mTotalLength;   //!< sum of all links' lengths
};

#endif // SerialRobot_h
