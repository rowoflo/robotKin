/*
 -------------------------------------------------------------------------------
 Robot.h
 robotTest Project
 
 CLASS NAME:
 Robot
 
 DESCRIPTION:
 description...
 
 FILES:
 Robot.h
 Robot.cpp
 
 DEPENDENCIES:
 
 
 CONSTRUCTORS:
 Robot();
 
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
 1.0 - 5/15/13 - Rowland O'Flaherty ( rowlandoflaherty.com )
 
 -------------------------------------------------------------------------------
 */



#ifndef _Robot_h_
#define _Robot_h_



//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include "Frame.h"
#include "Linkage.h"
#include <vector>
#include <map>
#include <string>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

//------------------------------------------------------------------------------
// Namespaces
//------------------------------------------------------------------------------
using namespace std;
using namespace Eigen;

namespace golems {
    
    
    //------------------------------------------------------------------------------
    // Typedefs
    //------------------------------------------------------------------------------
    typedef Matrix<double, 6, Dynamic> Matrix6Xd;
    
    
    class Robot : public Frame
    {
        //--------------------------------------------------------------------------
        // Robot Friends
        //--------------------------------------------------------------------------
        friend class Linkage;
        
        
    public:
        //--------------------------------------------------------------------------
        // Robot Lifecycle
        //--------------------------------------------------------------------------
        // Constructors
        Robot();
        Robot(vector<Linkage> linkageObjs, vector<int> parentIndices);
        
        // Destructor
        virtual ~Robot();

        
        //--------------------------------------------------------------------------
        // Robot Public Member Functions
        //--------------------------------------------------------------------------
        size_t nLinkages() const;
        
        size_t linkageIndex(string linkageName) const;
        
        const Linkage& const_linkage(size_t linkageIndex) const;
        const Linkage& const_linkage(string linkageName) const;
        
        Linkage& linkage(size_t linkageIndex);
        Linkage& linkage(string linkageName);
        
        const vector<Linkage>& const_linkages() const;
        vector<Linkage>& linkages();
        
        size_t nJoints() const;
        
        size_t jointIndex(string jointName) const;

        const Linkage::Joint* const_joint(size_t jointIndex) const;
        const Linkage::Joint* const_joint(string jointName) const;
        
        Linkage::Joint* joint(size_t jointIndex);
        Linkage::Joint* joint(string jointName);
        
        const vector<Linkage::Joint*>& const_joints() const;
        vector<Linkage::Joint*>& joints();
        
        VectorXd values() const;
        void values(const VectorXd& someValues);
        
        const Isometry3d& respectToFixed() const;
        void respectToFixed(Isometry3d aCoordinate);
        
        Isometry3d respectToWorld() const;
        
        void jacobian(MatrixXd& J, const vector<Linkage::Joint>& jointFrames, Vector3d location, const Frame* refFrame) const;
        
        void printInfo() const;
        
        
        
    protected:
        //--------------------------------------------------------------------------
        // Robot Protected Member Functions
        //--------------------------------------------------------------------------
        Isometry3d respectToWorld_; // Coordinates with respect to robot base frame
        vector<Linkage> linkages_;
        map<string, size_t> linkageNameToIndex_;
        vector<Linkage::Joint*> joints_;
        map<string, size_t> jointNameToIndex_;
        
        
        //--------------------------------------------------------------------------
        // Robot Protected Member Variables
        //--------------------------------------------------------------------------
        virtual void initialize(vector<Linkage> linkageObjs, vector<int> parentIndices);
        
        
    private:
        //--------------------------------------------------------------------------
        // Robot Constants, Enums, and Types
        //--------------------------------------------------------------------------
        void updateFrames();
        
        
        
        //--------------------------------------------------------------------------
        // Robot Private Member Variables
        //--------------------------------------------------------------------------
        bool initializing_;
        
        
        
    }; // class Robot
    
    //------------------------------------------------------------------------------
    // Postfix Increment Operators
    //------------------------------------------------------------------------------
    
} // namespace golems

#endif


