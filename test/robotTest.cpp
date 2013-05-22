/*
 -------------------------------------------------------------------------------
 robotTest.cpp
 robotTest Project
 
 Initially created by Rowland O'Flaherty ( rowlandoflaherty.com ) on 5/11/13.
 
 Version 1.0
 -------------------------------------------------------------------------------
 */



//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include <iostream>
#include <vector>
#include "Frame.h"
#include "Linkage.h"
#include "Robot.h"
#include "Hubo.h"



//------------------------------------------------------------------------------
// Namespaces
//------------------------------------------------------------------------------
using namespace std;
using namespace Eigen;
using namespace golems;

//------------------------------------------------------------------------------
// Global Varible Declarations
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
// Function Declarations
//------------------------------------------------------------------------------
void tutorial();


//------------------------------------------------------------------------------
// Main Function
//------------------------------------------------------------------------------

int main(int argc, char *argv[])
{
    tutorial();

    return 0;
}


//------------------------------------------------------------------------------
// Function Definitions
//------------------------------------------------------------------------------
void tutorial()
{
    cout << "---------------------------" << endl;
    cout << "|  Creating A Hubo Robot  |" << endl;
    cout << "---------------------------" << endl << endl;
    
    // Create an instance of a Hubo
    Hubo hubo;
    
    // Print info about hubo
    cout << "The robot " << hubo.name() << " consist of " << hubo.nLinkages() << " linkages and " << hubo.nJoints() << " joints all together." << endl << endl;
    
    
    cout << "-------------------------" << endl;
    cout << "|  Linkages and Joints  |" << endl;
    cout << "-------------------------" << endl << endl;
    
    
    cout << "The linkages are: " << endl;
    cout << "(ID, Name, Parent)" << endl;
    for (vector<Linkage>::iterator linkageIt = hubo.linkages().begin(); linkageIt != hubo.linkages().end(); ++linkageIt) {
        if (linkageIt->parentLinkage() == 0) {
            cout << linkageIt->id() << ", " << linkageIt->name() << " <- " << hubo.name() << endl;
        } else {
            cout << linkageIt->id() << ", " << linkageIt->name() << " <- " << linkageIt->parentLinkage()->name() << endl;
        }
    }
    cout << endl;
    
    size_t rightArmIndex = hubo.linkageIndex("RIGHT_ARM"); // Get index to right arm or can refer to right arm directly (see line below).
    cout << "Each linkage consist of joints." << endl << endl;
    cout << "For example the " << hubo.linkage(rightArmIndex).name() << " linkage has " << hubo.linkage(rightArmIndex).nJoints() << " joints." << endl << endl;
    
    cout << "The joints for the " << hubo.linkage(rightArmIndex).name() << " are:" << endl;
    cout << "(ID, Name, Value): " << endl;
    
    // Notice that there is a const_joints() as well as a joints() method (this exist for the robot class well), which allows for when the "this" pointer is const as is the case with the const iterator below.
    for (vector<Linkage::Joint>::const_iterator jointIt = hubo.linkage("RIGHT_ARM").const_joints().begin();
         jointIt != hubo.linkage("RIGHT_ARM").const_joints().end(); ++jointIt) {
        cout << jointIt->id() << ", " << jointIt->name() << ", " << jointIt->value() << endl;
    }
    cout << endl;
    
    cout << "---------------------------" << endl;
    cout << "|  Changing joint values  |" << endl;
    cout << "---------------------------" << endl << endl;
    
    cout << "Let's change the joint values." << endl << endl;
    
    // Set the joints all at once
    cout << "Set all the joints to have a value of PI/2." << endl;
    
    Linkage* rightArm = &hubo.linkage("RIGHT_ARM"); // Notice that this is a pointer
    VectorXd q = M_PI/2 * VectorXd::Ones(rightArm->nJoints(), 1);
    rightArm->values(q); // Set all joints at once
    
    cout << rightArm->values() << endl << endl;
    
    // Set the joints individually
    cout << "Set the joints individually." << endl;
    
    Linkage::Joint* joint0 = &rightArm->joint(0); // Again notice that this is a pointer
    Linkage::Joint joint1 = rightArm->joint(1); // This is not a pointer but return a const reference, but updating this joint will not update the robot
    Linkage::Joint* joint2 = &rightArm->joint("RSY"); // Just like linkages, joints can be index by name
    
    joint0->value(0.1);
    joint1.value(0.2);
    joint2->value(0.3);
    hubo.joint("REP")->value(0.4); // Notice that the joint() method of the robot class returns a pointer but the linkage() method returned a reference
    hubo.joint(23)->value(0.4); // Notice that the joint index in the robot is different than the joint index in the linkage
    
    cout << hubo.linkage("RIGHT_ARM").values() << endl << endl;
    
    // Reset all to position minus offset
    cout << "Reset joint values minus offsets." << endl;
    
    hubo.linkage("RIGHT_ARM").values(-hubo.rightArmOffsets);
    
    cout << hubo.linkage("RIGHT_ARM").values() << endl << endl;
    
    cout << "------------" << endl;
    cout << "|  Frames  |" << endl;
    cout << "------------" << endl << endl;
    
    cout << "Robots, Linkages, Joints, and Tools are all frames which have homogenous transformation with respect to differenct frames associated with them." << endl << endl;
    
    Linkage::Tool* rightHand = &rightArm->tool();
    cout << "For example the " << rightHand->name() << " tool at the end of the " << rightArm->name() << " linkage has the following homogenous transformations associated with it." << endl << endl;
    
    rightHand->printInfo();
    
    cout << "Every frame has a HG transformation with respect to a fixed frame and the world frame." << endl << endl;
    
    cout << "For example, the " << hubo.name() << " robot has the following: " << endl << endl;
    hubo.Frame::printInfo();
    
    cout << "Some types of frames (e.g. Linkage::Joint) have more HG transformations associated with their frame." << endl << endl;
    
    cout << "For example, the " << joint2->name() << " joint has the following: " << endl << endl;
    joint2->printInfo();
    
    cout << "These frames automatically get updated when the joint values change." << endl << endl;
    
    cout << "For example, let's move the right arm straight out in front of the robot." << endl << endl;
    cout << "Joint values of the " << rightArm->name() << " are:" << endl;
    joint0->value(-M_PI/2);
    cout << hubo.linkage("RIGHT_ARM").values() << endl << endl;
    
    cout << "The " << rightHand->name() << " tool now has the following values:" << endl;
    cout << "Respect to fixed frame (" << rightHand->parentJoint()->name() << " joint)" << endl;
    cout << rightHand->respectToFixed().matrix() << endl << endl;
    cout << "Respect to linkage frame (" << rightHand->parentLinkage()->name() << " linkage)" << endl;
    cout << rightHand->respectToLinkage().matrix() << endl << endl;
    cout << "Respect to robot frame (" << rightHand->parentRobot()->name() << " robot)" << endl;
    cout << rightHand->respectToRobot().matrix() << endl << endl;
    
    cout << "Notice the transform with respect to the fixed frame did not change but the others did." << endl << endl;
    
    cout << "Let's now twist the robot at the waist." << endl << endl;
    cout << "Joint values of the " << hubo.linkage("TORSO").name() << " are:" << endl;
    hubo.linkage("TORSO").joint(0).value(M_PI/2);
    cout << hubo.linkage("TORSO").values() << endl << endl;
    
    cout << "The " << rightHand->name() << " tool now has the following values:" << endl;
    cout << "Respect to fixed frame (" << rightHand->parentJoint()->name() << " joint)" << endl;
    cout << rightHand->respectToFixed().matrix() << endl << endl;
    cout << "Respect to linkage frame (" << rightHand->parentLinkage()->name() << " linkage)" << endl;
    cout << rightHand->respectToLinkage().matrix() << endl << endl;
    cout << "Respect to robot frame (" << rightHand->parentRobot()->name() << " robot)" << endl;
    cout << rightHand->respectToRobot().matrix() << endl << endl;
    
    cout << "Notice the transform with respect to the fixed frame and the linkage frame did not change but the with respect to the robot frame did." << endl << endl;
    
    cout << "---------------" << endl;
    cout << "|  Jacobians  |" << endl;
    cout << "---------------" << endl << endl;
    
    cout << "Jacobians for each linkage and the entire robot can be obtained easily." << endl << endl;
    
    cout << "For example, let's get the Jacobian for the " << rightArm->name() << " linkage where the location of the Jacobian is at " << rightArm->tool().name() << " and referenced with respect to the " << rightArm->name() << " linkage base coordinate frame." << endl << endl;
    rightArm->joint(0).value(0);
    cout << "Current joint values are:" << endl;
    cout << rightArm->values() << endl << endl;
    MatrixXd J;
    rightArm->jacobian(J,
                       rightArm->const_joints(),
                       rightArm->const_tool().respectToLinkage().translation(),
                       &hubo.linkage("RIGHT_ARM"));
    cout << "Jacobian is:" << endl;
    cout << J.matrix() << endl << endl;
    
    cout << "Or we can get the Jacobian from the " << hubo.linkage("RIGHT_LEG").tool().name() << " to the " << hubo.linkage("RIGHT_ARM").tool().name() << " and referenced with respect to " << hubo.name() << "." << endl << endl;
    
    vector<Linkage::Joint> joints;
    for (vector<Linkage::Joint>::reverse_iterator jointIt = hubo.linkage("RIGHT_LEG").joints().rbegin(); jointIt != hubo.linkage("RIGHT_LEG").joints().rend(); ++jointIt) {
        joints.push_back(*jointIt);
    }
    for (vector<Linkage::Joint>::iterator jointIt = hubo.linkage("TORSO").joints().begin(); jointIt != hubo.linkage("TORSO").joints().end(); ++jointIt) {
        joints.push_back(*jointIt);
    }
    for (vector<Linkage::Joint>::iterator jointIt = hubo.linkage("RIGHT_ARM").joints().begin(); jointIt != hubo.linkage("RIGHT_ARM").joints().end(); ++jointIt) {
        joints.push_back(*jointIt);
    }
    
    cout << "There are " << joints.size() << " joints envolved in this Jacobian with the values of" << endl;

    cout << "(Name, Value)" << endl;
    for (vector<Linkage::Joint>::iterator jointIt = joints.begin(); jointIt != joints.end(); ++jointIt) {
        cout << jointIt->name() << ", " << jointIt->value() << endl;
    }
    cout << endl;

     hubo.jacobian(J,
                   joints,
                   hubo.linkage("RIGHT_ARM").const_tool().respectToRobot().translation(),
                   &hubo);
    
    cout << "Jacobian is:" << endl;
    cout << J.matrix() << endl << endl;
    
    
    cout << "--------------------------------------" << endl;
    cout << "|  Analytical Arm Inverse Kinematics |" << endl;
    cout << "--------------------------------------" << endl << endl;
    
    cout << "Inverse kinematics of a linkage can also easily be obtained." << endl << endl;
    
    cout << "For example, let's set the joint values of the " << rightArm->name() << " to the following:" << endl;
    
    Isometry3d B0;
    VectorXd q0(6), q1(6);
    q0 <<
    -.1,
    -.2,
    -.3,
    -.4,
    -.5,
    -.6;
    
    rightArm->values(q0);
    
    cout << rightArm->values() << endl << endl;
    
    cout << "Then the " << rightArm->tool().name() << " with respect to the linkage has a pose of " << endl;
    
    B0 = rightArm->tool().respectToLinkage();
    
    cout << B0.matrix() << endl << endl;
    
    cout << "This HG transformation can be given to the IK to get the following joint values:" << endl;
    
    hubo.rightArmAnalyticalIK(q1, B0, q0);
    
    cout << q1 << endl << endl;
    
    
    cout << "------------------" << endl;
    cout << "|  Still to come |" << endl;
    cout << "------------------" << endl << endl;
    
    cout << "Things that still need to be added or fixed are the following:" << endl;
    cout << "-- Analytical Leg Inverse Kinematics" << endl;
    cout << "-- Numerical Inverse Kinematics" << endl;
    cout << "-- Documentation has to be written" << endl;
    cout << "-- Possibly fix how joints and linkages are stored so everything is reference by a pointer" << endl;
    cout << "-- Add methods to add and remove joints" << endl;
    
}



