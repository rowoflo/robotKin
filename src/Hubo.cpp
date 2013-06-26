/*
 -------------------------------------------------------------------------------
 Hubo.cpp
 robotTest Project
 
 Initially created by Rowland O'Flaherty ( rowlandoflaherty.com ) on 5/20/13.
 
 Version 1.0
 -------------------------------------------------------------------------------
 */



//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include "Hubo.h"



//------------------------------------------------------------------------------
// Namespaces
//------------------------------------------------------------------------------
using namespace std;
using namespace Eigen;
using namespace RobotKin;
using namespace Golems;


//------------------------------------------------------------------------------
// Hubo Lifecycle
//------------------------------------------------------------------------------
// Constructors
Hubo::Hubo()
:
zeroSize(1e-9)
{
    initialize();
}



// Destructor
Hubo::~Hubo()
{
    
}


//------------------------------------------------------------------------------
// Hubo Public Member Functions
//------------------------------------------------------------------------------
bool Hubo::leftArmAnalyticalIK(VectorXd& q, const Isometry3d& B, const VectorXd& qPrev)
{
    return armAnalyticalIK(q, B, qPrev, SIDE_LEFT);
}

bool Hubo::rightArmAnalyticalIK(VectorXd& q, const Isometry3d& B, const VectorXd& qPrev)
{
    return armAnalyticalIK(q, B, qPrev, SIDE_RIGHT);
}

bool Hubo::leftLegAnalyticalIK(VectorXd& q, const Isometry3d& B, const VectorXd& qPrev)
{
    return legAnalyticalIK(q, B, qPrev, SIDE_LEFT);
}

bool Hubo::rightLegAnalyticalIK(VectorXd& q, const Isometry3d& B, const VectorXd& qPrev)
{
    return legAnalyticalIK(q, B, qPrev, SIDE_RIGHT);
}


//------------------------------------------------------------------------------
// Hubo Private Member Functions
//------------------------------------------------------------------------------
void Hubo::initialize()
{
    // Lengths
    torsoLength = 0.1865;
    
    armLengths.resize(4, 1);
    armLengths <<
        0.21450,                // neck -> shoulder Y
        0.17914,                // shoulder -> elbow Z
        0.18159,                // elbow -> wrist Z
        0.12065;                // wrist -> ee Z
    
    legLengths.resize(5, 1);
    legLengths <<
        0.0884,                 // waist -> hip Y
        0.1825,                 // waist -> hip Z
        0.3000,                 // hip -> knee Z
        0.3000,                 // knee -> ankle Z
        0.0950;                 // ankle to foot Z
    
    // Limits
    torsoLimits.resize(1, 2);
    torsoLimits <<
    -M_PI/2, M_PI/2;        // torso yaw
    
    leftArmLimits.resize(6, 2);
    leftArmLimits <<
        -2.0, 2.0,              // shoulder pitch
        -0.3, 2.0,              // shoulder roll
        -2.0, 2.0,              // shoulder yaw
        -2.0, 0.01,             // elbow pitch
        -2.0, 2.0,              // wrist yaw
        -1.4, 1.2;              // wrist pitch
    
    rightArmLimits.resize(6, 2);
    rightArmLimits <<
        -2.0, 2.0,              // shoulder pitch
        -2.0, 0.3,              // shoulder roll
        -2.0, 2.0,              // shoulder yaw
        -2.0, 0.01,             // elbow pitch
        -2.0, 2.0,              // wrist yaw
        -1.4, 1.2;              // wrist pitch
    
    leftLegLimits.resize(6, 2);
    leftLegLimits <<
        -M_PI/2,    M_PI/2,     // hip yaw
        -0.488692,  0.488692,   // hip roll
        -1.48353,   1.6057,     // hip pitch
        -0.0698132, 2.60054,    // knee pitch
        -1.29154,   1.69297,    // ankle pitch
        -0.191986,  0.191986;   // ankle roll
    
    rightLegLimits.resize(6, 2);
    rightLegLimits <<
        -M_PI/2,    M_PI/2,     // hip yaw
        -0.488692,  0.488692,   // hip roll
        -1.48353,   1.6057,     // hip pitch
        -0.0698132, 2.60054,    // knee pitch
        -1.29154,   1.69297,    // ankle pitch
        -0.191986,  0.191986;   // ankle roll
    
    // Offsets
    torsoOffset = 0.0;         // torso yaw
    
    leftArmOffsets.resize(6);
    leftArmOffsets <<
        0.0,                    // shoulder pitch
        0.3,                    // shoulder roll
        0.0,                    // shoulder yaw
        0.0,                    // elbow pitch
        0.0,                    // wrist yaw
        0.0;                    // wrist pitch
    
    
    rightArmOffsets.resize(6);
    rightArmOffsets <<
        0.0,                    // shoulder pitch
       -0.3,                    // shoulder roll
        0.0,                    // shoulder yaw
        0.0,                    // elbow pitch
        0.0,                    // wrist yaw
        0.0;                    // wrist pitch
    
    leftLegOffsets.resize(6);
    leftLegOffsets <<
        0.0,                    // hip yaw
        0.0,                    // hip roll
        0.0,                    // hip pitch
        0.0,                    // knee pitch
        0.0,                    // ankle pitch
        0.0;                    // ankle roll
    
    rightLegOffsets.resize(6);
    rightLegOffsets <<
        0.0,                    // hip yaw
        0.0,                    // hip roll
        0.0,                    // hip pitch
        0.0,                    // knee pitch
        0.0,                    // ankle pitch
        0.0;                    // ankle roll
    
    
    // Linkage torso
    Linkage torso = initializeTorso();
    
    // Linkage left arm;
    Linkage leftArm = initializeLeftArm();
    
    // Linkage right arm;
    Linkage rightArm = initializeRightArm();
    
    // Linkage left leg;
    Linkage leftLeg = initializeLeftLeg();
    
    // Linkage right leg;
    Linkage rightLeg = initializeRightLeg();
    
    // Robot
    vector<Linkage> linkages(5);
    
    linkages[0] = torso;
    linkages[1] = leftArm;
    linkages[2] = rightArm;
    linkages[3] = leftLeg;
    linkages[4] = rightLeg;
    
    
    vector<int> parentIndices(5);
    parentIndices[0] = -1;
    parentIndices[1] = 0;
    parentIndices[2] = 0;
    parentIndices[3] = -1;
    parentIndices[4] = -1;
    
//    parentIndices = {-1, 0, 0, -1, -1};
    Robot::initialize(linkages, parentIndices);
    name("HUBO");
}


Linkage Hubo::initializeTorso()
{    
    Matrix<double, 4, 4> T;
    Isometry3d jointFrame;
    
    T <<
    1.0,  0.0,  0.0,  0.0,
    0.0,  1.0,  0.0,  0.0,
    0.0,  0.0,  1.0,  torsoLength,
    0.0,  0.0,  0.0,  1.0;
    jointFrame = T;
    
    Linkage::Joint joint(jointFrame, "TOR");
    
    Linkage torso(Isometry3d::Identity(), "TORSO", 0, joint);
    
    return torso;
}


Linkage Hubo::initializeLeftArm()
{
    Matrix<double, 4, 4> T;
    size_t nJoints = 6;
    
    vector<Isometry3d> jointFrames;
    jointFrames.resize(nJoints);
    vector<string> jointNames;
    jointNames.resize(nJoints);
    
    vector<Linkage::Joint> joints;
    joints.resize(nJoints);
    
    
    T <<
    1.0,  0.0,  0.0,  0.0,
    0.0,  0.0,  1.0,  armLengths(0),
    0.0, -1.0,  0.0,  0.0,
    0.0,  0.0,  0.0,  1.0;
    jointFrames[0] = T;
    jointNames[0] = "LSP";
    
    T <<
    0.0,  0.0,  1.0,  0.0,
    1.0,  0.0,  0.0,  0.0,
    0.0,  1.0,  0.0,  0.0,
    0.0,  0.0,  0.0,  1.0;
    jointFrames[1] = T;
    jointNames[1] = "LSR";
    
    T <<
    0.0,  0.0, -1.0,  0.0,
    -1.0,  0.0,  0.0,  0.0,
    0.0,  1.0,  0.0,  0.0,
    0.0,  0.0,  0.0,  1.0;
    jointFrames[2] = T;
    jointNames[2] = "LSY";
    
    T <<
    0.0,  0.0, -1.0,  0.0,
    1.0,  0.0,  0.0,  0.0,
    0.0, -1.0,  0.0, -armLengths(1),
    0.0,  0.0,  0.0,  1.0;
    jointFrames[3] = T;
    jointNames[3] = "LEP";
    
    T <<
    1.0,  0.0,  0.0,  0.0,
    0.0,  0.0, -1.0,  0.0,
    0.0,  1.0,  0.0,  0.0,
    0.0,  0.0,  0.0,  1.0;
    jointFrames[4] = T;
    jointNames[4] = "LEY";
    
    T <<
    1.0,  0.0,  0.0,  0.0,
    0.0,  0.0,  1.0,  0.0,
    0.0, -1.0,  0.0, -armLengths(2),
    0.0,  0.0,  0.0,  1.0;
    jointFrames[5] = T;
    jointNames[5] = "LWP";
    
    for (size_t i = 0; i < nJoints; ++i) {
        joints[i].respectToFixed(jointFrames[i] * Eigen::AngleAxisd(leftArmOffsets[i], Eigen::Vector3d::UnitZ()) );
        joints[i].name(jointNames[i]);
    }
    
    T <<
    0.0,  0.0,  1.0,  0.0,
    1.0,  0.0,  0.0,  armLengths(3),
    0.0,  1.0,  0.0,  0.0,
    0.0,  0.0,  0.0,  1.0;
    
    Isometry3d toolFrame(T);
    Linkage::Tool tool(toolFrame, "LEFT_HAND");
    
    Linkage leftArm(Isometry3d::Identity(), "LEFT_ARM", 0, joints, tool);
    
    return leftArm;
}


Linkage Hubo::initializeRightArm()
{
    Matrix<double, 4, 4> T;
    size_t nJoints = 6;
    
    vector<Isometry3d> jointFrames;
    jointFrames.resize(nJoints);
    vector<string> jointNames;
    jointNames.resize(nJoints);
    
    vector<Linkage::Joint> joints;
    joints.resize(nJoints);
    
    
    T <<
    1.0,  0.0,  0.0,  0.0,
    0.0,  0.0,  1.0,  -armLengths(0),
    0.0, -1.0,  0.0,  0.0,
    0.0,  0.0,  0.0,  1.0;
    jointFrames[0] = T;
    jointNames[0] = "RSP";
    
    T <<
    0.0,  0.0,  1.0,  0.0,
    1.0,  0.0,  0.0,  0.0,
    0.0,  1.0,  0.0,  0.0,
    0.0,  0.0,  0.0,  1.0;
    jointFrames[1] = T;
    jointNames[1] = "RSR";
    
    T <<
    0.0,  0.0, -1.0,  0.0,
    -1.0,  0.0,  0.0,  0.0,
    0.0,  1.0,  0.0,  0.0,
    0.0,  0.0,  0.0,  1.0;
    jointFrames[2] = T;
    jointNames[2] = "RSY";
    
    T <<
    0.0,  0.0, -1.0,  0.0,
    1.0,  0.0,  0.0,  0.0,
    0.0, -1.0,  0.0, -armLengths(1),
    0.0,  0.0,  0.0,  1.0;
    jointFrames[3] = T;
    jointNames[3] = "REP";
    
    T <<
    1.0,  0.0,  0.0,  0.0,
    0.0,  0.0, -1.0,  0.0,
    0.0,  1.0,  0.0,  0.0,
    0.0,  0.0,  0.0,  1.0;
    jointFrames[4] = T;
    jointNames[4] = "REY";
    
    T <<
    1.0,  0.0,  0.0,  0.0,
    0.0,  0.0,  1.0,  0.0,
    0.0, -1.0,  0.0, -armLengths(2),
    0.0,  0.0,  0.0,  1.0;
    jointFrames[5] = T;
    jointNames[5] = "RWP";
    
    for (size_t i = 0; i < nJoints; ++i) {
        joints[i].respectToFixed(jointFrames[i] * Eigen::AngleAxisd(rightArmOffsets[i], Eigen::Vector3d::UnitZ()) );
        joints[i].name(jointNames[i]);
    }    
    
    T <<
    0.0,  0.0,  1.0,  0.0,
    1.0,  0.0,  0.0,  armLengths(3),
    0.0,  1.0,  0.0,  0.0,
    0.0,  0.0,  0.0,  1.0;
    
    Isometry3d toolFrame(T);
    Linkage::Tool tool(toolFrame, "RIGHT_HAND");
    
    Linkage rightArm(Isometry3d::Identity(), "RIGHT_ARM", 0, joints, tool);
    
    return rightArm;
}


Linkage Hubo::initializeLeftLeg()
{
    Matrix<double, 4, 4> T;
    size_t nJoints = 6;
    
    vector<Isometry3d> jointFrames;
    jointFrames.resize(nJoints);
    vector<string> jointNames;
    jointNames.resize(nJoints);
    
    vector<Linkage::Joint> joints;
    joints.resize(nJoints);
    
    
    T <<
    1.0,  0.0,  0.0,  0.0,
    0.0,  1.0,  0.0,  legLengths(0),
    0.0,  0.0,  1.0, -legLengths(1),
    0.0,  0.0,  0.0,  1.0;
    jointFrames[0] = T;
    jointNames[0] = "LHY";
    
    T <<
    0.0,  0.0,  1.0,  0.0,
    1.0,  0.0,  0.0,  0.0,
    0.0,  1.0,  0.0,  0.0,
    0.0,  0.0,  0.0,  1.0;
    jointFrames[1] = T;
    jointNames[1] = "LHR";
    
    T <<
    0.0,  0.0,  1.0,  0.0,
   -1.0,  0.0,  0.0,  0.0,
    0.0, -1.0,  0.0,  0.0,
    0.0,  0.0,  0.0,  1.0;
    jointFrames[2] = T;
    jointNames[2] = "LHP";
    
    T <<
    1.0,  0.0,  0.0,  legLengths(2),
    0.0,  1.0,  0.0,  0.0,
    0.0,  0.0,  1.0,  0.0,
    0.0,  0.0,  0.0,  1.0;
    jointFrames[3] = T;
    jointNames[3] = "LKP";
    
    T <<
    1.0,  0.0,  0.0,  legLengths(3),
    0.0,  1.0,  0.0,  0.0,
    0.0,  0.0,  1.0,  0.0,
    0.0,  0.0,  0.0,  1.0;
    jointFrames[4] = T;
    jointNames[4] = "LAP";
    
    T <<
    1.0,  0.0,  0.0,  0.0,
    0.0,  0.0, -1.0,  0.0,
    0.0,  1.0,  0.0,  0.0,
    0.0,  0.0,  0.0,  1.0;
    jointFrames[5] = T;
    jointNames[5] = "LAR";
    
    for (size_t i = 0; i < nJoints; ++i) {
        joints[i].respectToFixed(jointFrames[i] * Eigen::AngleAxisd(leftLegOffsets[i], Eigen::Vector3d::UnitZ()) );
        joints[i].name(jointNames[i]);
    }
    
    T <<
    1.0,  0.0,  0.0,  legLengths(4),
    0.0,  1.0,  0.0,  0.0,
    0.0,  0.0,  1.0,  0.0,
    0.0,  0.0,  0.0,  1.0;
    
    Isometry3d toolFrame(T);
    Linkage::Tool tool(toolFrame, "LEFT_FOOT");
    
    Linkage leftLeg(Isometry3d::Identity(), "LEFT_LEG", 0, joints, tool);
    
    return leftLeg;    
}


Linkage Hubo::initializeRightLeg()
{
    Matrix<double, 4, 4> T;
    size_t nJoints = 6;
    
    vector<Isometry3d> jointFrames;
    jointFrames.resize(nJoints);
    vector<string> jointNames;
    jointNames.resize(nJoints);
    
    vector<Linkage::Joint> joints;
    joints.resize(nJoints);
    
    
    T <<
    1.0,  0.0,  0.0,  0.0,
    0.0,  1.0,  0.0, -legLengths(0),
    0.0,  0.0,  1.0, -legLengths(1),
    0.0,  0.0,  0.0,  1.0;
    jointFrames[0] = T;
    jointNames[0] = "RHY";
    
    T <<
    0.0,  0.0,  1.0,  0.0,
    1.0,  0.0,  0.0,  0.0,
    0.0,  1.0,  0.0,  0.0,
    0.0,  0.0,  0.0,  1.0;
    jointFrames[1] = T;
    jointNames[1] = "RHR";
    
    T <<
    0.0,  0.0,  1.0,  0.0,
    -1.0,  0.0,  0.0,  0.0,
    0.0, -1.0,  0.0,  0.0,
    0.0,  0.0,  0.0,  1.0;
    jointFrames[2] = T;
    jointNames[2] = "RHP";
    
    T <<
    1.0,  0.0,  0.0,  legLengths(2),
    0.0,  1.0,  0.0,  0.0,
    0.0,  0.0,  1.0,  0.0,
    0.0,  0.0,  0.0,  1.0;
    jointFrames[3] = T;
    jointNames[3] = "RKP";
    
    T <<
    1.0,  0.0,  0.0,  legLengths(3),
    0.0,  1.0,  0.0,  0.0,
    0.0,  0.0,  1.0,  0.0,
    0.0,  0.0,  0.0,  1.0;
    jointFrames[4] = T;
    jointNames[4] = "RAP";
    
    T <<
    1.0,  0.0,  0.0,  0.0,
    0.0,  0.0, -1.0,  0.0,
    0.0,  1.0,  0.0,  0.0,
    0.0,  0.0,  0.0,  1.0;
    jointFrames[5] = T;
    jointNames[5] = "RAR";
    
    for (size_t i = 0; i < nJoints; ++i) {
        joints[i].respectToFixed(jointFrames[i] * Eigen::AngleAxisd(rightLegOffsets[i], Eigen::Vector3d::UnitZ()) );
        joints[i].name(jointNames[i]);
    }
    
    T <<
    1.0,  0.0,  0.0,  legLengths(4),
    0.0,  1.0,  0.0,  0.0,
    0.0,  0.0,  1.0,  0.0,
    0.0,  0.0,  0.0,  1.0;
    
    Isometry3d toolFrame(T);
    Linkage::Tool tool(toolFrame, "RIGHT_FOOT");
    
    Linkage rightLeg(Isometry3d::Identity(), "RIGHT_LEG", 0, joints, tool);
    
    return rightLeg;
}


void Hubo::armFK(Isometry3d& B, const Vector6d& q, size_t side)
{
    size_t index;
    if (side == SIDE_RIGHT) {
        index = linkageIndex("RIGHT_ARM");
    } else {
        index = linkageIndex("LEFT_ARM");
    }
    VectorXd q0 = linkage(index).values();
    linkage(index).values(q);
    B = linkage(index).tool().respectToLinkage();
    linkage(index).values(q0);
}

void Hubo::legFK(Isometry3d& B, const Vector6d& q, size_t side)
{
    size_t index;
    if (side == SIDE_RIGHT) {
        index = linkageIndex("RIGHT_LEG");
    } else {
        index = linkageIndex("LEFT_LEG");
    }
    VectorXd q0 = linkage(index).values();
    linkage(index).values(q);
    B = linkage(index).tool().respectToLinkage();
    linkage(index).values(q0);
}

bool Hubo::armAnalyticalIK(VectorXd& q, const Isometry3d& B, const Vector6d& qPrev, size_t side)
{
    q.resize(6,1);
    Eigen::ArrayXXd qAll(6,8);
    
    // Declarations
    Isometry3d shoulder, shoulderInv, toolFixed, toolFixedInv, B5_6, BInv;
    double nx, sx, ax, px;
    double ny, sy, ay, py;
    double nz, sz, az, pz;
    double q1, q2, q3, q4, q5, q6;
    double qP1, qP3;
    double qT;
    Eigen::Matrix<int, 8, 3> m;
    
    double S2, S4, S5, S6;
    double C2, C4, C5, C6;
    
    Eigen::MatrixXd limits(6,2);
    Vector6d offsets; offsets.setZero();
    
    // Parameters
    //double l1 = armLengths[0];
    double l2 = armLengths[1];
    double l3 = armLengths[2];
    double l4 = armLengths[3];
    
    // Variables
    if (side == SIDE_RIGHT) {
        // Transformation from Neck frame to right shoulder pitch frame
        shoulder = const_linkage("RIGHT_ARM").const_joint("RSP").respectToFixed();
        
        // Tool relative to last joint
        toolFixed = const_linkage("RIGHT_ARM").const_tool().respectToFixed();
        
        
        limits = rightArmLimits;
        offsets = rightArmOffsets;

        
    } else {
        // Transformation from Neck frame to left shoulder pitch frame
        shoulder = const_linkage("LEFT_ARM").const_joint("LSP").respectToFixed();
        
        // Tool relative to last joint
        toolFixed = const_linkage("LEFT_ARM").const_tool().respectToFixed();
        
        limits = leftArmLimits;
        offsets = leftArmOffsets;
        
    }
    // This is needed bc the IK was developed for the tool being specified to the hand and not the last joint orginally.
    Matrix<double, 4, 4> T;
    T <<
    0.0, -1.0,  0.0,  0.0,
    1.0,  0.0,  0.0,  l4,
    0.0,  0.0,  1.0,  0.0,
    0.0,  0.0,  0.0,  1.0;
    B5_6 = T;
    
    shoulderInv = shoulder.inverse();
    toolFixedInv = toolFixed.inverse();    
    
    BInv = (shoulderInv*B*toolFixedInv*B5_6).inverse();
    
    nx = BInv(0,0); sx = BInv(0,1); ax = BInv(0,2); px = BInv(0,3);
    ny = BInv(1,0); sy = BInv(1,1); ay = BInv(1,2); py = BInv(1,3);
    nz = BInv(2,0); sz = BInv(2,1); az = BInv(2,2); pz = BInv(2,3);   
    
    qP1 = qPrev(0); qP3 = qPrev(2);
    
    m <<
    1,  1,  1,
    1,  1, -1,
    1, -1,  1,
    1, -1, -1,
    -1,  1,  1,
    -1,  1, -1,
    -1, -1,  1,
    -1, -1, -1;
    
    // Calculate inverse kinematics
    for (size_t i = 0; i < 8; i++) {
        
        // Solve for q4
        C4 = max(min((2*l4*px - l2*l2 - l3*l3 + l4*l4 + px*px + py*py + pz*pz)/(2*l2*l3),1.0),-1.0);

        if (fabs(C4 - 1) < zeroSize) { // Case 1: q4 == 0
            // Set q4
            q4 = 0;
            
            // Set q3
            q3 = qP3;
            
            // Solve for q6
            S6 = max(min( py/(l2 + l3), 1.0),-1.0);
            C6 = max(min( -(l4 + px)/(l2 + l3), 1.0), -1.0);
            q6 = atan2(S6,C6);
            
            
            // Solve for q2
            S2 = max(min( C4*C6*ax - C4*S6*ay, 1.0),-1.0);
            if (fabs(S2 - 1) < zeroSize) {
                q2 = M_PI/2;
            } else if (fabs(S2 + 1) < zeroSize) {
                q2 = -M_PI/2;
            } else {
                complex<double> radical = 1-S2*S2;
                q2 = atan2(S2,m(i,2)*real(sqrt(radical)));
            }
            
            // Solve for q5
            qT = atan2(-C6*ay - S6*ax,az);
            C2 = cos(q2);
            
            if (fabs(C2) < zeroSize) { // Case 3: q2 = pi/2 or -pi/2
                
                q1 = qP1;
                q3 = qP3;
                
                // Solve for q5
                if (S2 > 0) { // Case 3a: q2 = pi/2
                    qT = atan2(nz,-sz);
                    q5 = wrapToPi(q1 - q3 - qT);
                } else { // Case 3b: q2 = -pi/2
                    qT = atan2(-nz,sz);
                    q5 = wrapToPi(qT - q1 - q3);
                }
                
                
            } else {
                
                if (C2 < 0) {
                    qT = qT + M_PI;
                }
                q5 = wrapToPi(qT - q3);
                
                // Solve for q1
                q1 = atan2(S6*ny - C6*nx,C6*sx - S6*sy);
                if (C2 < 0) {
                    q1 = q1 + M_PI;
                }
                q1 = wrapToPi(q1);
            }
            
        } else {
            
            // Solve for q4
            complex<double> radical = 1-C4*C4;
            q4 = atan2(m(i,0)*real(sqrt(radical)),C4);
            
            // Solve for q5
            S4 = sin(q4);
            S5 = pz/(S4*l2);
            if (fabs(S5 - 1) < zeroSize) {
                q5 = M_PI/2;
            } else if (fabs(S5 + 1) < zeroSize) {
                q5 = -M_PI/2;
            } else {
                radical = 1-S5*S5;
                q5 = atan2(S5,m(i,1)*real(sqrt(radical)));
            }
            
            // Solve for q6
            C5 = cos(q5);
            S6 =max(min( (C5*S4*l2 + (py*(l3 + C4*l2 - (C5*S4*l2*py)/(l4 + px)))/(l4 + px + py*py/(l4 + px)))/(l4 + px), 1.0),-1.0);
            C6 = max(min( -(l3 + C4*l2 - (C5*S4*l2*py)/(l4 + px))/(l4 + px + py*py/(l4 + px)), 1.0),-1.0);
            q6 = atan2(S6,C6);
            
            // Solve for q2
            S2 = max(min(ax*(C4*C6 - C5*S4*S6) - ay*(C4*S6 + C5*C6*S4) - S4*S5*az,1.0),-1.0);
            if (fabs(S2 - 1) < zeroSize) {
                q2 = M_PI/2;
            } else if (fabs(S2 + 1) < zeroSize) {
                q2 = -M_PI/2;
            } else {
                radical = 1-S2*S2;
                q2 = atan2(S2,m(i,2)*real(sqrt(radical)));
            }
            
            // Solve for q3
            C2 = cos(q2);
            
            if (fabs(C2) < zeroSize) { // Case 2: q2 = pi/2 or -pi/2
                
                q3 = qP3;
                // Solve for q1
                if (S2 > 0) { // Case 2a: q2 = pi/2
                    qT = atan2(S6*sy - C6*sx,S6*ny - C6*nx);
                    if (S4 < 0) {
                        qT = qT + M_PI;
                    }
                    q1 = wrapToPi(qT + q3);
                } else { // Case 2b: q2 = -pi/2
                    qT = atan2(S6*sy - C6*sx,S6*ny - C6*nx);
                    if (S4 < 0) {
                        qT = qT + M_PI;
                    }
                    q1 = wrapToPi(qT - q3);
                }
                
            } else {
                q3 = atan2(S4*S6*ay - C4*S5*az - C6*S4*ax - C4*C5*C6*ay - C4*C5*S6*ax,C5*az - C6*S5*ay - S5*S6*ax);
                if (C2 < 0) {
                    q3 = q3 - M_PI;
                }
                q3 = wrapToPi(q3);
                
                // Solve for q1
                q1 = atan2(C4*S6*ny - C4*C6*nx + S4*S5*nz + C5*C6*S4*ny + C5*S4*S6*nx,C4*C6*sx - C4*S6*sy - S4*S5*sz - C5*C6*S4*sy - C5*S4*S6*sx);
                if (C2 < 0) {
                    q1 = q1 + M_PI;
                }
                q1 = wrapToPi(q1);
            }
        }
        
        qAll(0,i) = q1;
        qAll(1,i) = q2;
        qAll(2,i) = q3;
        qAll(3,i) = q4;
        qAll(4,i) = q5;
        qAll(5,i) = q6;
        
    }

    // Set to offsets
    for( size_t j=0; j<8; j++) {
        for (size_t i = 0; i < 6; i++) {
            if (side==SIDE_RIGHT) {
                qAll(i,j) = wrapToPi(qAll(i,j) - offsets(i));
            } else {
                qAll(i,j) = wrapToPi(qAll(i,j) - offsets(i));
            }
        }
    }
    // TODO: Find best solution using better method
    
    Eigen::ArrayXd qDiff(6,1); qDiff.setZero();
    Eigen::ArrayXd qDiffSum(8,1);
    bool withinLim[8];
    size_t minInd;
    
    // if any joint solution is infintesimal, set it to zero
    for(size_t i=0; i<8; i++)
        for(size_t j=0; j<6; j++)
            if(qAll(j,i) < zeroSize && qAll(j,i) > -zeroSize)
                qAll(j,i) = 0.0;
    
    // Initialize withinLim to all trues for all eight solutions
    for(size_t i=0; i<8; i++)
        withinLim[i] = true;
    
    // Check each set of solutions to see if any are outside the limits
    for(size_t i=0; i<8; i++)
        for(size_t j=0; j<6; j++)
            if( limits(j,0) > qAll(j,i) || qAll(j,i) > limits(j,1) )
                withinLim[i] = false;
    
    // Initialze anyWithin boolean array to all trues
    bool anyWithin=false;
    for(size_t i=0; i<8; i++)
        if( withinLim[i] )
            anyWithin = true;
    
    // If any solution has all joints within the limits...
    if(anyWithin)
    {
        // for each solution...
        for (size_t i = 0; i < 8; i++) {
            // if all the joints of solution i are within the limits...
            if( withinLim[i] )
            {
                // calculate the differences between solution angles, j, and previous angles
                for (size_t j=0; j < 6; j++)
                    qDiff(j) = wrapToPi(qAll(j,i) - qPrev(j));
                // sum the absolute values of the differences to get total difference
                qDiffSum(i) = qDiff.abs().sum();
            }
            // if the solution doesn't have all the joints within the limits...
            else
                // set the difference for that solution to infinity
                qDiffSum(i) = std::numeric_limits<double>::infinity();
        }
        // and take the solution closest to previous solution
        qDiffSum.minCoeff(&minInd);
        q = qAll.col(minInd);
    }
    // if no solution has all the joints within the limits...
    else
    {
        // then for each solution...
        for( size_t i=0; i<8; i++)
        {
            // create a 6d vector of angles of solution i
            Vector6d qtemp = qAll.col(i).matrix();
            // take the min of the angles and the joint upper limits
            qtemp = qtemp.cwiseMin(limits.col(1));
            // then take the max of those angles and the joint lower limits
            qtemp = qtemp.cwiseMax(limits.col(0));
            // create an Isometry3d 4x4 matrix for the temp pose
            Isometry3d Btemp;
            // find the pose associated with the temp angles
            armFK( Btemp, qtemp, side );
            // calculate the distance from previous pose to temp pose locations
            qDiffSum(i) = (Btemp.translation() - B.translation()).norm();
        }
        // find the solution that's closest the previous position
        qDiffSum.minCoeff(&minInd);
        q = qAll.col(minInd);
    }
    // apply limits
    for( size_t i=0; i<6; i++ )
        q(i) = max( min( q(i), limits(i,1)), limits(i,0) );
    
    return anyWithin;

}


bool Hubo::legAnalyticalIK(VectorXd& q, const Isometry3d& B, const Vector6d& qPrev, size_t side) {
    Eigen::ArrayXXd qAll(6,8);
    
    // Declarations
    Isometry3d neck, neckInv, waist, waistInv, BInv;
    double nx, sx, ax, px;
    double ny, sy, ay, py;
    double nz, sz, az, pz;
    double q1, q2, q3, q4, q5, q6;
    double C45, psi, q345;
    Eigen::Matrix<int, 8, 3> m;
    
    double S2, S4, S6;
    double C2, C4, C5, C6;
    
    Eigen::MatrixXd limits(6,2);
    Vector6d offsets; offsets.setZero();
    
    // Parameters
    //    double l1 = (79.5+107)/1000.0;
    double l2 = legLengths[0];
    double l3 = legLengths[1];
    double l4 = legLengths[2];
    double l5 = legLengths[3];
    double l6 = legLengths[4];
    
    if (side == SIDE_RIGHT) {
        // Transformation from Waist frame to right hip yaw frame
        waist(0,0) = 0; waist(0,1) = -1; waist(0,2) = 0; waist(0,3) =   0;
        waist(1,0) = 1; waist(1,1) =  0; waist(1,2) = 0; waist(1,3) = -l2;
        waist(2,0) = 0; waist(2,1) =  0; waist(2,2) = 1; waist(2,3) = -l3;
        waist(3,0) = 0; waist(3,1) =  0; waist(3,2) = 0; waist(3,3) =   1;
        
        limits = rightLegLimits;
        offsets = rightLegOffsets;
        
    } else {
        // Transformation from Waist frame to left hip yaw frame
        waist(0,0) = 0; waist(0,1) = -1; waist(0,2) = 0; waist(0,3) =   0;
        waist(1,0) = 1; waist(1,1) =  0; waist(1,2) = 0; waist(1,3) =  l2;
        waist(2,0) = 0; waist(2,1) =  0; waist(2,2) = 1; waist(2,3) = -l3;
        waist(3,0) = 0; waist(3,1) =  0; waist(3,2) = 0; waist(3,3) =   1;

        limits = leftLegLimits;
        offsets = leftLegOffsets;
        
    }
    waistInv = waist.inverse();
    
    // Variables
    BInv = (waistInv*B).inverse();
    
    nx = BInv(0,0); sx = BInv(0,1); ax = BInv(0,2); px = BInv(0,3);
    ny = BInv(1,0); sy = BInv(1,1); ay = BInv(1,2); py = BInv(1,3);
    nz = BInv(2,0); sz = BInv(2,1); az = BInv(2,2); pz = BInv(2,3);
    
    m <<
    1,  1,  1,
    1,  1, -1,
    1, -1,  1,
    1, -1, -1,
    -1,  1,  1,
    -1,  1, -1,
    -1, -1,  1,
    -1, -1, -1;
    
    for (size_t i = 0; i < 8; i++)
    {
        C4 = ((l6 + px)*(l6 + px) - l4*l4 - l5*l5 + py*py + pz*pz)/(2*l4*l5);
        complex<double> radical = 1-C4*C4;
        q4 = atan2(m(i,0)*real(sqrt(radical)),C4);
        
        S4 = sin(q4);
        psi = atan2(S4*l4, C4*l4+l5);
        radical = ((px+l6)*(px+l6)+(py*py));
        q5 = wrapToPi(atan2(-pz, m(i,1)*real(sqrt(radical)))-psi);
        
        q6 = atan2(py, -px-l6);
        C45 = cos(q4+q5);
        C5 = cos(q5);
        if (C45*l4 + C5*l5 < 0)
        {
            q6 = wrapToPi(q6 + M_PI);
        }
        
        S6 = sin(q6);
        C6 = cos(q6);
        
        S2 = C6*ay + S6*ax;
        radical = 1-S2*S2;
        q2 = atan2(S2,m(i,2)*real(sqrt(radical)));
        
        q1 = atan2(C6*sy + S6*sx,C6*ny + S6*nx);
        C2 = cos(q2);
        if (C2 < 0) {
            q1 = wrapToPi(q1 + M_PI);
        }
        
        q345 = atan2(-az/C2,-(C6*ax - S6*ay)/C2);
        q3 = wrapToPi(q345-q4-q5);
        
        qAll(0,i) = q1;
        qAll(1,i) = q2;
        qAll(2,i) = q3;
        qAll(3,i) = q4;
        qAll(4,i) = q5;
        qAll(5,i) = q6;
    }
    
    // Set to offset
    for (size_t i = 0; i < 6; i++) {
        if (side==SIDE_RIGHT) {
            q(i) = wrapToPi(q(i) + offsets(i));
        } else {
            q(i) = wrapToPi(q(i) + offsets(i));
        }
    }
    
    // Find best solution
    Eigen::ArrayXd qDiff(6,1); qDiff.setZero();
    Eigen::ArrayXd qDiffSum(8,1);
    bool withinLim[8];
    size_t minInd;
    
    // if any joint solution is infintesimal, set it to zero
    for(size_t i=0; i<8; i++)
        for(size_t j=0; j<6; j++)
            if(qAll(j,i) < zeroSize && qAll(j,i) > -zeroSize)
                qAll(j,i) = 0.0;
    
    // Initialize withinLim to all trues for all eight solutions
    for(size_t i=0; i<8; i++)
        withinLim[i] = true;
    
    // Check each set of solutions to see if any are outside the limits
    for(size_t i=0; i<8; i++)
        for(size_t j=0; j<6; j++)
            if( limits(j,0) > qAll(j,i) || qAll(j,i) > limits(j,1) )
                withinLim[i] = false;
    
    // Initialze anyWithin boolean array to all trues
    bool anyWithin=false;
    for(size_t i=0; i<8; i++)
        if( withinLim[i] )
            anyWithin = true;
    
    // If any solution has all joints within the limits...
    if(anyWithin)
    {
        // for each solution...
        for (size_t i = 0; i < 8; i++)
        {
            // if all the joints of solution i are within the limits...
            if( withinLim[i] )
            {
                // calculate the differences between solution angles, j, and previous angles
                for (size_t j=0; j < 6; j++)
                {
                    qDiff(j) = wrapToPi(qAll(j,i) - qPrev(j));
                }
                // sum the absolute values of the differences to get total difference
                qDiffSum(i) = qDiff.abs().sum();
            }
            // if the solution doesn't have all the joints within the limits...
            else
                // set the difference for that solution to infinity
                qDiffSum(i) = std::numeric_limits<double>::infinity();
        }
        // and take the solution closest to previous solution
        qDiffSum.minCoeff(&minInd);
        q = qAll.col(minInd);
    }
    
    // if no solution has all the joints within the limits...
    else
    {
        // then for each solution...
        for(size_t i=0; i<8; i++)
        {
            // create a 6d vector of angles of solution i
            Vector6d qtemp = qAll.col(i).matrix();
            // take the min of the angles and the joint upper limits
            qtemp = qtemp.cwiseMin(limits.col(1));
            // then take the max of those angles and the joint lower limits
            qtemp = qtemp.cwiseMax(limits.col(0));
            // create an Isometry3d 4x4 matrix for the temp pose
            Isometry3d Btemp;
            // find the pose associated with the temp angles
            legFK( Btemp, qtemp, side );
            // calculate the distance from previous pose to temp pose locations
            qDiffSum(i) = (Btemp.translation() - B.translation()).norm();
        }
        // find the solution that's closest the previous position
        qDiffSum.minCoeff(&minInd);
        q = qAll.col(minInd);
    }
    // set the final joint angles to the solution closest to the previous solution
    for( size_t i=0; i<6; i++)
        q(i) = max( min( q(i), limits(i,1)), limits(i,0) );
    
    return anyWithin;
}


