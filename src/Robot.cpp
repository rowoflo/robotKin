/*
 -------------------------------------------------------------------------------
 Robot.cpp
 robotTest Project
 
 Initially created by Rowland O'Flaherty ( rowlandoflaherty.com ) on 5/15/13.
 
 Version 1.0
 -------------------------------------------------------------------------------
 */



//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include "Robot.h"

//------------------------------------------------------------------------------
// Namespaces
//------------------------------------------------------------------------------
using namespace golems;


//------------------------------------------------------------------------------
// Robot Lifecycle
//------------------------------------------------------------------------------
// Constructors
Robot::Robot()
:
Frame::Frame(Isometry3d::Identity()),
respectToWorld_(Isometry3d::Identity()),
initializing_(false)
{
    frameType_ = ROBOT;
}

Robot::Robot(vector<Linkage> linkageObjs, vector<int> parentIndices)
:
Frame::Frame(Isometry3d::Identity()),
respectToWorld_(Isometry3d::Identity()),
initializing_(false)
{
    frameType_ = ROBOT;
    
    initialize(linkageObjs, parentIndices);
}


// Destructor
Robot::~Robot()
{
    
}


//--------------------------------------------------------------------------
// Robot Public Member Functions
//--------------------------------------------------------------------------
size_t Robot::nLinkages() const { return linkages_.size(); }

size_t Robot::linkageIndex(string linkageName) const { return linkageNameToIndex_.at(linkageName); }

const Linkage& Robot::const_linkage(size_t linkageIndex) const
{
    assert(linkageIndex < nLinkages());
    return linkages_[linkageIndex];
}
const Linkage& Robot::const_linkage(string linkageName) const { return linkages_[linkageNameToIndex_.at(linkageName)]; }

Linkage& Robot::linkage(size_t linkageIndex)
{
    assert(linkageIndex < nLinkages());
    return linkages_[linkageIndex];
}
Linkage& Robot::linkage(string linkageName) { return linkages_[linkageNameToIndex_.at(linkageName)]; }

const vector<Linkage>& Robot::const_linkages() const { return linkages_; }

vector<Linkage>& Robot::linkages() { return linkages_; }

size_t Robot::nJoints() const { return joints_.size(); }

size_t Robot::jointIndex(string jointName) const { return jointNameToIndex_.at(jointName); }

const Linkage::Joint* Robot::const_joint(size_t jointIndex) const
{
    assert(jointIndex < nJoints());
    return joints_[jointIndex];
}
const Linkage::Joint* Robot::const_joint(string jointName) const { return joints_[jointNameToIndex_.at(jointName)]; }

Linkage::Joint* Robot::joint(size_t jointIndex)
{
    assert(jointIndex < nJoints());
    return joints_[jointIndex];
}
Linkage::Joint* Robot::joint(string jointName) { return joints_[jointNameToIndex_.at(jointName)]; }

const vector<Linkage::Joint*>& Robot::const_joints() const { return joints_; }

vector<Linkage::Joint*>& Robot::joints() { return joints_; }

VectorXd Robot::values() const
{
    VectorXd theValues(nJoints(),1);
    for (size_t i = 0; i < nJoints(); ++i) {
        theValues[i] = joints_[i]->value();
    }
    return theValues;
}

void Robot::values(const VectorXd& someValues) {
    assert(someValues.size() == nJoints());
    for (size_t i = 0; i < nJoints(); ++i) {
        joints_[i]->value(someValues(i));
    }
    updateFrames();
}


const Isometry3d& Robot::respectToFixed() const { return respectToFixed_; };
void Robot::respectToFixed(Isometry3d aCoordinate)
{
    respectToFixed_ = aCoordinate;
    updateFrames();
}

Isometry3d Robot::respectToWorld() const
{
    return respectToWorld_;
}

void Robot::jacobian(MatrixXd& J, const vector<Linkage::Joint>& jointFrames, Vector3d location, const Frame* refFrame) const
{ // location should be specified in respect to robot coordinates
    size_t nCols = jointFrames.size();
    J.resize(6, nCols);
    
    Vector3d o_i, d_i, z_i; // Joint i location, offset, axis
    
    for (size_t i = 0; i < nCols; i++) {
        
        o_i = jointFrames[i].respectToRobot().translation(); // Joint i location
        d_i = o_i - location; // VEctor from location to joint i
        z_i = jointFrames[i].respectToRobot().rotation().col(2); // Joint i joint axis
        
        // Set column i of Jocabian
        if (jointFrames[i].jointType_ == REVOLUTE) {
            J.block(0, i, 3, 1) = d_i.cross(z_i);
            J.block(3, i, 3, 1) = z_i;
        } else {
            J.block(0, i, 3, 1) = z_i;
            J.block(3, i, 3, 1) = Vector3d::Zero();
        }
        
    }
    
    // Jacobian transformation
    Matrix3d r(refFrame->respectToWorld().rotation().inverse() * respectToWorld_.rotation());
    MatrixXd R(6,6);
    R << r, Matrix3d::Zero(), Matrix3d::Zero(), r;
    J = R * J;
}

void Robot::printInfo() const
{
    Frame::printInfo();
    
    cout << "Linkages (ID, Name <- Parent): " << endl;
    for (vector<Linkage>::const_iterator linkageIt = const_linkages().begin();
         linkageIt != const_linkages().end(); ++linkageIt) {
        if (linkageIt->parentLinkage_ == 0) {
            cout << linkageIt->id() << ", " << linkageIt->name() << " <- " << this->name() << endl;
        } else {
            cout << linkageIt->id() << ", " << linkageIt->name() << " <- " << linkageIt->parentLinkage_->name() << endl;
        }
    }
    cout << "Joints (ID, Name, Value): " << endl;
    for (vector<Linkage::Joint*>::const_iterator jointIt = const_joints().begin();
         jointIt != const_joints().end(); ++jointIt) {
        cout << (*jointIt)->id() << ", " << (*jointIt)->name() << ", " << (*jointIt)->value() << endl;
    }
    for (vector<Linkage>::const_iterator linkageIt = const_linkages().begin();
         linkageIt != const_linkages().end(); ++linkageIt) {
        linkageIt->printInfo();
    }
}

//------------------------------------------------------------------------------
// Robot Protected Member Functions
//------------------------------------------------------------------------------
void Robot::initialize(vector<Linkage> linkages, vector<int> parentIndices)
{
    initializing_ = true;
    
    assert(linkages.size() == parentIndices.size());
    
    vector<indexParentIndexPair> iPI(linkages.size());
    for (size_t i = 0; i != linkages.size(); ++i) {
        iPI[i].I = i;
        iPI[i].pI = parentIndices[i];
    }
    sort(iPI.begin(), iPI.end());
    
    
    // Initialize
    int nJoints = 0;
    for (size_t i = 0; i != linkages.size(); ++i) {
        linkages_.push_back(linkages[iPI[i].I]);
        linkages_[i].id_ = i;
        linkageNameToIndex_[linkages_[i].name_] = i;
        nJoints += linkages_[i].nJoints();
    }
    
    size_t jointCnt = 0;
    for (size_t i = 0; i != linkages_.size(); ++i) {
        if (parentIndices[iPI[i].I] == -1) {
            linkages_[i].parentLinkage_ = 0;
        } else {
            linkages_[i].parentLinkage_ = &(linkages_[parentIndices[iPI[i].I]]);
            linkages_[parentIndices[iPI[i].I]].childLinkages_.push_back(&(linkages_[i]));
        }
        for (size_t j = 0; j != linkages_[i].nJoints(); ++j) {
            linkages_[i].joints_[j].linkage_ = &(linkages_[i]);
            linkages_[i].joints_[j].robot_ = this;
            joints_.push_back(&(linkages_[i].joints_[j]));
            joints_.back()->id_ = jointCnt;
            jointNameToIndex_[joints_[jointCnt]->name()] = jointCnt;
            jointCnt++;
        }
        linkages_[i].tool_.linkage_ = &(linkages_[i]);
        linkages_[i].tool_.robot_ = this;
        linkages_[i].robot_ = this;
        linkages_[i].tool_.id_ = i;
    }
    
    initializing_ = false;
    
    updateFrames();
}


//------------------------------------------------------------------------------
// Robot Private Member Functions
//------------------------------------------------------------------------------
void Robot::updateFrames()
{
    if (~initializing_) {
        for (vector<Linkage>::iterator linkageIt = linkages_.begin();
             linkageIt != linkages_.end(); ++linkageIt) {
            
            if (linkageIt->parentLinkage_ == 0) {
                linkageIt->respectToRobot_ = linkageIt->respectToFixed_;
            } else {
                linkageIt->respectToRobot_ = linkageIt->parentLinkage_->tool_.respectToRobot() * linkageIt->respectToFixed_;
            }
        }
    }
}




