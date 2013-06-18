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
using namespace RobotKin;


//------------------------------------------------------------------------------
// Robot Lifecycle
//------------------------------------------------------------------------------
// Constructors
Robot::Robot()
        : Frame::Frame(Isometry3d::Identity()),
          respectToWorld_(Isometry3d::Identity()),
          initializing_(false)
{
    linkages_.resize(0);
    frameType_ = ROBOT;
}

Robot::Robot(vector<Linkage> linkageObjs, vector<int> parentIndices)
        : Frame::Frame(Isometry3d::Identity()),
          respectToWorld_(Isometry3d::Identity()),
          initializing_(false)
{
    frameType_ = ROBOT;
    
    linkages_.resize(0);
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
    
    if(linkages.size() != parentIndices.size())
    {
        std::cerr << "ERROR! Number of linkages (" << linkages.size() << ") does not match "
                  << "the number of parent indices (" << parentIndices.size() << ")!"
                  << std::endl;
        return;
    }
    
    cerr << "Sorting linkages" << endl;
    vector<indexParentIndexPair> iPI(linkages.size());
    for (size_t i = 0; i != linkages.size(); ++i) {
        iPI[i].I = i;
        iPI[i].pI = parentIndices[i];
    }
    sort(iPI.begin(), iPI.end());
    // ^^^ Make sure that parents get added before children
    
    
    initializing_ = true;
    
    /*
    linkages_.reserve(100);
    cerr << "Adding linkages" << endl;
    for(size_t i = 0; i != linkages.size(); ++i)
    {
        cerr << "Adding " << linkages[iPI[i].I].name() << endl;
        addLinkage(linkages[iPI[i].I], parentIndices[iPI[i].I], linkages[iPI[i].I].name());
        cerr << "Finished adding" << endl;
    }
    */
    
    
    // Initialize
    
    size_t jointCnt = 0;
    for (size_t i = 0; i != linkages.size(); ++i) {
        
        linkages_.push_back(linkages[iPI[i].I]);
        linkages_[i].id_ = i;
        linkageNameToIndex_[linkages_[i].name_] = i;
        
        
        if (parentIndices[iPI[i].I] == -1) {
            linkages_[i].parentLinkage_ = 0;
        } else {
            linkages_[i].parentLinkage_ = &(linkages_[parentIndices[iPI[i].I]]);
            linkages_[parentIndices[iPI[i].I]].childLinkages_.push_back(&(linkages_[i]));
        }
        for (size_t j = 0; j != linkages_[i].nJoints(); ++j) {
            linkages_[i].joints_[j].linkage_ = &(linkages_[i]);
            linkages_[i].joints_[j].hasLinkage = true;
            linkages_[i].joints_[j].robot_ = this;
            linkages_[i].joints_[j].hasRobot = true;
            joints_.push_back(&(linkages_[i].joints_[j]));
            joints_.back()->id_ = jointCnt;
            jointNameToIndex_[joints_[jointCnt]->name()] = jointCnt;
            jointCnt++;
        }
        linkages_[i].tool_.linkage_ = &(linkages_[i]);
        linkages_[i].tool_.hasLinkage = true;
        linkages_[i].tool_.robot_ = this;
        linkages_[i].tool_.hasRobot = true;
        linkages_[i].robot_ = this;
        linkages_[i].hasRobot = true;
        linkages_[i].tool_.id_ = i;
    }
    

    // Deprecated (I hope)
    /*
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
            linkages_[i].joints_[j].hasLinkage = true;
            linkages_[i].joints_[j].robot_ = this;
            linkages_[i].joints_[j].hasRobot = true;
            joints_.push_back(&(linkages_[i].joints_[j]));
            joints_.back()->id_ = jointCnt;
            jointNameToIndex_[joints_[jointCnt]->name()] = jointCnt;
            jointCnt++;
        }
        linkages_[i].tool_.linkage_ = &(linkages_[i]);
        linkages_[i].tool_.hasLinkage = true;
        linkages_[i].tool_.robot_ = this;
        linkages_[i].tool_.hasRobot = true;
        linkages_[i].robot_ = this;
        linkages_[i].hasRobot = true;
        linkages_[i].tool_.id_ = i;
    }
    */
    
    initializing_ = false;
    
    
    updateFrames();
}

void Robot::addLinkage(Linkage linkage, int parentIndex, string name)
{
    if(parentIndex == -1)
        linkage.parentLinkage_ = NULL;
    else if( parentIndex > linkages_.size()-1 )
    {
        std::cerr << "ERROR! Parent index value (" << parentIndex << ") is larger "
                  << "than the current highest linkage index (" << linkages().size()-1 << ")!"
                  << std::endl;
        return;
    }
    else
        linkage.parentLinkage_ = &(linkages_[parentIndex]);
    
    linkage.hasParent = true; // TODO: Decide if this should be true for root linkage or not
    
    
    // Get the linkage adjusted to its new home
    size_t newIndex = linkages_.size();
    cerr << "Adding linkage #" << newIndex;
    linkages_.push_back(linkage);
    linkages_[newIndex].robot_ = this;
//    linkages_[newIndex].hasRobot = true;
    linkages_[newIndex].id_ = newIndex;
    linkages_[newIndex].name_ = name;
    // Move in its luggage
    for(size_t j = 0; j != linkages_[newIndex].nJoints(); ++j)
    {
        cerr << " Joint:" << j;
        linkages_[newIndex].joints_[j].linkage_ = &(linkages_[newIndex]);
//        linkages_[newIndex].joints_[j].hasLinkage = true;
        linkages_[newIndex].joints_[j].robot_ = this;
//        linkages_[newIndex].joints_[j].hasRobot = true;
        joints_.push_back(&(linkages_[newIndex].joints_[j]));
        joints_.back()->id_ = joints_.size()-1;
        jointNameToIndex_[joints_.back()->name()] = joints_.size()-1;
    }
    cerr << endl;
    // TODO: Allow for multiple tools
    linkages_[newIndex].tool_.linkage_ = &(linkages_[newIndex]);
//    linkages_[newIndex].tool_.hasLinkage = true;
    linkages_[newIndex].tool_.robot_ = this;
//    linkages_[newIndex].tool_.hasRobot = true;
    linkages_[newIndex].tool_.id_ = newIndex;
    
    // Tell the post office we've moved in
    linkageNameToIndex_[linkages_[newIndex].name_] = newIndex;
    
    // Inform the parent of its pregnancy
    if(linkages_[newIndex].parentLinkage_ != NULL)
    {
        linkages_[newIndex].parentLinkage_->childLinkages_.push_back(&linkages_[newIndex]);
        linkages_[newIndex].hasChildren = true;
    }
}

void Robot::addLinkage(int parentIndex, string name)
{
    Linkage dummyLinkage;
    addLinkage(dummyLinkage, parentIndex, name);
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




