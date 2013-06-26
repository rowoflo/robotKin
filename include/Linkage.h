/*
 -------------------------------------------------------------------------------
 Linkage.h
 Robot Library Project
 
 CLASS NAME:
 Linkage
 
 DESCRIPTION:
 description...
 
 FILES:
 Linkage.h
 Linkage.cpp
 
 DEPENDENCIES:
 
 
 CONSTRUCTORS:
 Linkage();
 
 PROPERTIES:
 prop1 - description... .
 
 prop2 - description... .
 
 METHODS:
 type method1(type arg1);
 Description... .
 arg1 - description... .
 
 NOTES:
 
 
 EXAMPLES:
 Example 1: description
 ----------------------------------------------------------------------------
 code...
 ----------------------------------------------------------------------------
 
 
 VERSIONS:
 1.0 - 5/11/13 - Rowland O'Flaherty ( rowlandoflaherty.com )
 
 -------------------------------------------------------------------------------
 */



#ifndef _Linkage_h_ 
#define _Linkage_h_



//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include "Frame.h"
#include <math.h>
#include <string>
#include <vector>
#include <map>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

//------------------------------------------------------------------------------
// Class Declarations
//------------------------------------------------------------------------------
namespace RobotKin
{
    class Robot;
}

//------------------------------------------------------------------------------
// Namespaces
//------------------------------------------------------------------------------
using namespace std;
using namespace Eigen;

namespace RobotKin
{
    
    
    //------------------------------------------------------------------------------
    // Typedefs and Enums
    //------------------------------------------------------------------------------
    typedef Eigen::Matrix<double, 6, Dynamic> Matrix6Xd;
    
    enum JointType {
        REVOLUTE,
        PRISMATIC,
    };
    
    
    class Linkage : public Frame
    {
        //--------------------------------------------------------------------------
        // Linkage Friends
        //--------------------------------------------------------------------------
        friend class Joint;
        friend class Tool;
        friend class Robot;
        
    public:
        //--------------------------------------------------------------------------
        // Linkage Nested Classes
        //--------------------------------------------------------------------------
        class Joint : public Frame
        {
            
            //----------------------------------------------------------------------
            // Joint Friends
            //----------------------------------------------------------------------
            friend class Linkage;
            friend class Robot;
            
        public:
            
            //----------------------------------------------------------------------
            // Joint Lifecycle
            //----------------------------------------------------------------------
            // Constructors
            Joint(Isometry3d respectToFixed = Isometry3d::Identity(),
                  string name = "",
                  size_t id = 0,
                  JointType jointType = REVOLUTE,
                  double minValue = -M_PI,
                  double maxValue = M_PI,
                  double minVel = -INFINITY,
                  double maxVel = INFINITY);
            
            // Destructor
            virtual ~Joint();
            
            //----------------------------------------------------------------------
            // Joint Overloaded Operators
            //----------------------------------------------------------------------
            // Assignment operator
            const Linkage::Joint& operator=(double value);
            
            //----------------------------------------------------------------------
            // Joint Public Member Functions
            //----------------------------------------------------------------------
            double value() const;
            void value(double value);

            double vel() const;
            void vel(double vel);            

            double max() const;
            void max(double max);

            double min() const;
            void min(double min);

            double maxVel() const;
            void maxVel(double maxVel);

            double minVel() const;
            void minVel(double minVel);
            
            const Isometry3d& respectToFixed() const;
            void respectToFixed(Isometry3d aCoordinate);
            
            const Isometry3d& respectToFixedTransformed() const;
            
            const Isometry3d& respectToLinkage() const;
            
            Isometry3d respectToRobot() const;
            
            Isometry3d respectToWorld() const;
            
            const Linkage* linkage() const;
            
            void printInfo() const; 
            
        protected:
            //----------------------------------------------------------------------
            // Joint Protected Member Variables
            //----------------------------------------------------------------------
            double value_; // Current joint value (R type = joint angle, P type = joint length)
            double vel_; // Current joint velocity
            
        private:
            //----------------------------------------------------------------------
            // Joint Private Member Variables
            //----------------------------------------------------------------------
            JointType jointType_; // Type of joint (REVOLUTE or PRISMATIC)
            double min_; // Minimum joint value
            double max_; // Maximum joint value
            double minVel_; // Minimum joint velocity
            double maxVel_; // Maximum joint velocity

            Isometry3d respectToFixedTransformed_; // Coordinates transformed according to the joint value and type with respect to respectToFixed frame
            Isometry3d respectToLinkage_; // Coordinates with respect to linkage base frame
            Linkage* linkage_;
            Robot* robot_;
            
        }; // class Joint
        
        class Tool : public Frame
        {
            
            //----------------------------------------------------------------------
            // Tool Friends
            //----------------------------------------------------------------------
            friend class Linkage;
            friend class Robot;
            
        public:
            //----------------------------------------------------------------------
            // Tool Lifecycle
            //----------------------------------------------------------------------
            // Constructors
            Tool(Isometry3d respectToFixed = Isometry3d::Identity(),
                 string name = "",
                 size_t id = 0);
            
            // Destructor
            virtual ~Tool();
            
            //----------------------------------------------------------------------
            // Tool Public Member Functions
            //----------------------------------------------------------------------
            const Isometry3d& respectToFixed() const;
            void respectToFixed(Isometry3d aCoordinate);
            
            const Isometry3d& respectToLinkage() const;
            
            Isometry3d respectToRobot() const;
            
            Isometry3d respectToWorld() const;
            
            const Linkage::Joint* parentJoint() const;
            
            const Linkage* parentLinkage() const;
            
            const Robot* parentRobot() const;
            
            void printInfo() const;
            
            static Linkage::Tool Identity();
            
            
        private:
            //----------------------------------------------------------------------
            // Tool Private Member Variables
            //----------------------------------------------------------------------
            Isometry3d respectToLinkage_; // Coordinates with respect to linkage base frame
            Linkage::Joint* joint_;
            Linkage* linkage_;
            Robot* robot_;
            
        }; // class Tool
        
        
        //--------------------------------------------------------------------------
        // Linkage Lifecycle
        //--------------------------------------------------------------------------
        // Constructors
        Linkage();
        Linkage(Isometry3d respectToFixed,
                string name, size_t id,
                Linkage::Joint joint,
                Linkage::Tool tool = Linkage::Tool::Identity());
        Linkage(Isometry3d respectToFixed,
                string name,
                size_t id,
                vector<Linkage::Joint> joints,
                Linkage::Tool tool = Linkage::Tool::Identity());
        
        // Destructor
        virtual ~Linkage();
        
        
        //--------------------------------------------------------------------------
        // Linkage Overloaded Operators
        //--------------------------------------------------------------------------
        // Assignment operator
        const Linkage& operator=(const VectorXd& values);
        
        
        //--------------------------------------------------------------------------
        // Linkage Public Member Functions
        //--------------------------------------------------------------------------
        
        Linkage* parentLinkage();
        
        size_t nChildren() const;
        
        
        size_t nJoints() const;
        const Linkage::Joint& const_joint(size_t jointIndex) const;
        const Linkage::Joint& const_joint(string jointName) const;
        
        Linkage::Joint& joint(size_t jointIndex);
        Linkage::Joint& joint(string jointName);
        
        const vector<Linkage::Joint>& const_joints() const;
        vector<Linkage::Joint>& joints();
        
        const Linkage::Tool& const_tool() const;
        Linkage::Tool& tool();
        
        VectorXd values() const;
        void values(const VectorXd &someValues);

        VectorXd vels() const;
        void vels(const VectorXd &someVels);

        VectorXd maxVels() const;
        VectorXd minVels() const;
        
        const Isometry3d& respectToFixed() const;
        void respectToFixed(Isometry3d aCoordinate);
        
        const Isometry3d& respectToRobot() const;
        
        Isometry3d respectToWorld() const;
        
        void jacobian(MatrixXd& J, const vector<Linkage::Joint>& jointFrames, Vector3d location, const Frame* refFrame) const;
        
        void printInfo() const;
        
        
        //--------------------------------------------------------------------------
        // Linkage Public Member Variables
        //--------------------------------------------------------------------------
        bool (*analyticalIK)(VectorXd& q, const Isometry3d& B, const VectorXd& qPrev);
        
    protected:        
        //--------------------------------------------------------------------------
        // Linkage Protected Member Variables
        //--------------------------------------------------------------------------
        Isometry3d respectToRobot_; // Coordinates with respect to robot base frame
        Robot* robot_;
        Linkage* parentLinkage_;
        vector<Linkage*> childLinkages_;
        vector<Linkage::Joint> joints_;
        Linkage::Tool tool_;
     
        
    private:
        //--------------------------------------------------------------------------
        // Linkage Public Member Functions
        //--------------------------------------------------------------------------
        void initialize(vector<Linkage::Joint> joints, Linkage::Tool tool);
        void updateFrames();
        void updateChildLinkage();
        static bool defaultAnalyticalIK(VectorXd& q, const Isometry3d& B, const VectorXd& qPrev);
        
        
        //--------------------------------------------------------------------------
        // Linkage Private Member Variables
        //--------------------------------------------------------------------------
        bool initializing_;
        map<string, size_t> jointNameToIndex_;
        
        
    }; // class Linkage

    
} // namespace RobotKin

#endif


