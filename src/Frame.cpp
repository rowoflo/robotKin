/*
 -------------------------------------------------------------------------------
 Frame.cpp
 Robot Library Project
 
 Initially created by Rowland O'Flaherty ( rowlandoflaherty.com ) on 5/11/13.
 
 Version 1.0
 -------------------------------------------------------------------------------
 */



//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include "Frame.h"



//------------------------------------------------------------------------------
// Namespaces
//------------------------------------------------------------------------------
using namespace RobotKin;
using namespace std;


//------------------------------------------------------------------------------
// Lifecycle
//------------------------------------------------------------------------------
// Constructors
Frame::Frame(Isometry3d respectToFixed, string name, size_t id, FrameType frameType)
:
name_(name),
id_(id),
frameType_(frameType),
respectToFixed_(respectToFixed)
{
    
}

// Destructor
Frame::~Frame()
{
    
}


//------------------------------------------------------------------------------
// Frame Public Member Functions
//------------------------------------------------------------------------------
size_t Frame::id() const { return id_; }

string Frame::name() const { return name_; }
void Frame::name(string name) { name_ = name; }

FrameType Frame::frameType() const { return frameType_; }
string Frame::frameTypeString() const
{
    switch (frameType_) {
        case UNKNOWN:
            return string("Unknown");
            break;
            
        case JOINT:
            return string("Joint");
            break;
            
        case TOOL:
            return string("Tool");
            break;
            
        case LINKAGE:
            return string("Linkage");
            break;
            
        case ROBOT:
            return string("Robot");
            break;
            
        default:
            break;
    }
}

Isometry3d Frame::respectTo(const Frame* aFrame) const
{
    return aFrame->respectToWorld().inverse() * respectToWorld();
}

void Frame::printInfo() const
{
    cout << frameTypeString() << " Frame Info: " << name() << " (ID: " << id() << ")" << endl;
    cout << "Respect to fixed frame:" << endl;
    cout << respectToFixed().matrix() << endl << endl;
    cout << "Respect to world frame:" << endl;
    cout << respectToWorld().matrix() << endl<< endl;
}



