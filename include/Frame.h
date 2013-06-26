/*
 -------------------------------------------------------------------------------
 Frame.h
 Robot Library Project
 
 CLASS NAME:
 Frame
 
 DESCRIPTION:
 description...
 
 FILES:
 Frame.h
 Frame.cpp
 
 DEPENDENCIES:
 
 
 CONSTRUCTORS:
 Frame();
 
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



#ifndef _Frame_h_
#define _Frame_h_



//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include <string>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>


//------------------------------------------------------------------------------
// Namespaces
//------------------------------------------------------------------------------
using namespace std;
using namespace Eigen;


namespace RobotKin
{
    
    
    //------------------------------------------------------------------------------
    // Typedefs
    //------------------------------------------------------------------------------
    
    enum FrameType {
        UNKNOWN,
        JOINT,
        TOOL,
        LINKAGE,
        ROBOT
    };
    
    
    
    class Frame
    {
        
        //--------------------------------------------------------------------------
        // Frame Friends
        //--------------------------------------------------------------------------
        friend class Linkage;
        friend class Robot;
        
    public:
        //--------------------------------------------------------------------------
        // Frame Destructor
        //--------------------------------------------------------------------------
        virtual ~Frame();
        
        
        //--------------------------------------------------------------------------
        // Frame Public Member Functions
        //--------------------------------------------------------------------------
        size_t id() const;
        
        string name() const;
        void name(string name);
        
        FrameType frameType() const;
        string frameTypeString() const;
        
        virtual const Isometry3d& respectToFixed() const = 0;
        virtual void respectToFixed(Isometry3d aCoordinate) = 0;
        
        virtual Isometry3d respectToWorld() const = 0;

        Isometry3d respectTo(const Frame* aFrame) const;
        
        virtual void printInfo() const;
        
        
    protected:
        //--------------------------------------------------------------------------
        // Frame Constructor
        //--------------------------------------------------------------------------
        Frame(Isometry3d respectToFixed = Isometry3d::Identity(),
              string name = "",
              size_t id = 0,
              FrameType frameType = UNKNOWN);
        
        //--------------------------------------------------------------------------
        // Frame Protected Member Variables
        //--------------------------------------------------------------------------
        string name_;
        size_t id_;
        FrameType frameType_;
        Isometry3d respectToFixed_; // Coordinates with respect to some fixed frame in nominal position
        
        
    }; // clase Frame
    
} // namespace RobotKin

#endif


