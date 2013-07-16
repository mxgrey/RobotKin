/*
 -------------------------------------------------------------------------------
 Linkage.cpp
 Robot Library Project
 
 Initially created by Rowland O'Flaherty ( rowlandoflaherty.com ) on 5/11/13.
 
 Version 1.0
 -------------------------------------------------------------------------------
 */



//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include "Linkage.h"
#include "Robot.h"


//------------------------------------------------------------------------------
// Namespaces
//------------------------------------------------------------------------------
using namespace std;
using namespace Eigen;
using namespace RobotKin;


//------------------------------------------------------------------------------
// Linkage Nested Classes
//------------------------------------------------------------------------------
// Joint Class Constructors

Joint& Joint::operator =( const Joint& joint )
{
    respectToFixed_ = joint.respectToFixed_;
    respectToFixedTransformed_ = joint.respectToFixedTransformed_;

    name_ = joint.name_;
    id_ = joint.id_; // TODO: Should id really be changed?
    frameType_ = joint.frameType_;

    jointType_ = joint.jointType_;
    jointAxis_ = joint.jointAxis_;
    min_ = joint.min_;
    max_ = joint.max_;

    value(joint.value_);
}

Joint::Joint(const Joint &joint)
    : Frame::Frame(joint.respectToFixed_, joint.name(), joint.id(), JOINT),
      respectToFixedTransformed_(joint.respectToFixedTransformed_),
      jointType_(joint.jointType_),
      jointAxis_(joint.jointAxis_),
      min_(joint.min_),
      max_(joint.max_)
{
    value(joint.value_);
}

Joint::Joint(TRANSFORM respectToFixed,
                      string name,
                      size_t id,
                      JointType jointType,
                      AXIS axis,
                      double minValue, double maxValue)
            : Frame::Frame(respectToFixed, name, id, JOINT),
              respectToFixedTransformed_(respectToFixed),
              respectToLinkage_(respectToFixed),
              jointType_(jointType),
              min_(minValue),
              max_(maxValue),
              value_(0)
{
    setJointAxis(axis);
    value(value_);
}


// Joint Destructor
Joint::~Joint()
{

}

// Joint Overloaded Operators
const Joint& Joint::operator=(const double aValue)
{
    value(aValue);
    return *this;
}

void Joint::setJointAxis(AXIS axis)
{
    jointAxis_ = axis;
    jointAxis_.normalize();
}

// Joint Methods
double Joint::value() const { return value_; }
void Joint::value(double newValue)
{
    if(newValue < min_)
        value_ = min_;
    else if(newValue > max_)
        value_ = max_;
    else
        value_ = newValue;
    value_ = newValue;

    if (jointType_ == REVOLUTE) {
        respectToFixedTransformed_ = respectToFixed_ * Eigen::AngleAxisd(value_, jointAxis_);
    } else if(jointType_ == PRISMATIC){
        respectToFixedTransformed_ = respectToFixed_ * Eigen::Translation3d(value_*jointAxis_);
    } else {
        respectToFixedTransformed_ = respectToFixed_;
    }

    // TODO: Decide if it is efficient to have this here
    if ( hasLinkage )
        linkage_->updateFrames();
}

double Joint::min() const { return min_; }
void Joint::min(double newMin)
{
    min_ = newMin;

    if(max_ < min_)
        max_ = min_;

    if(value_ < min_)
        value_ = min_;
}

double Joint::max() const { return max_; }
void Joint::max(double newMax)
{
    max_ = newMax;

    if(min_ > max_)
        min_ = max_;

    if(value_ > max_)
        value_ = max_;
}



const TRANSFORM& Joint::respectToFixed() const { return respectToFixed_; }
void Joint::respectToFixed(TRANSFORM aCoordinate)
{
    respectToFixed_ = aCoordinate;
    value(value_);
}

const TRANSFORM& Joint::respectToFixedTransformed() const
{
    return respectToFixedTransformed_;
}

const TRANSFORM& Joint::respectToLinkage() const
{
    return respectToLinkage_;
}

TRANSFORM Joint::respectToRobot() const
{
    if(hasLinkage)
        return linkage_->respectToRobot_ * respectToLinkage_;
    else
        return TRANSFORM::Identity();
}

TRANSFORM Joint::respectToWorld() const
{
    if(hasLinkage)
        return linkage_->respectToWorld() * respectToLinkage_;
    else
        return TRANSFORM::Identity();
}

const Linkage* Joint::linkage() const
{
    return linkage_;
}

void Joint::printInfo() const
{
    cout << frameTypeString() << " Info: " << name() << " (ID: " << id()  << "), Joint Type:"
         << jointType_ << endl;
    cout << "Joint value: " << value() << "\t Axis: " << jointAxis_.transpose() << endl;
    cout << "Respect to fixed frame:" << endl;
    cout << respectToFixed().matrix() << endl << endl;
    cout << "Respect to fixed after transformation: " << endl;
    cout << respectToFixedTransformed().matrix() << endl << endl;
    cout << "Respect to linkage frame:" << endl;
    cout << respectToLinkage().matrix() << endl << endl;
    cout << "Respect to robot frame:" << endl;
    cout << respectToRobot().matrix() << endl << endl;
    cout << "Respect to world frame:" << endl;
    cout << respectToWorld().matrix() << endl << endl;
}


// Tool Class
Tool& Tool::operator =(const Tool& tool)
{
    respectToFixed_ = tool.respectToFixed_;
    respectToLinkage_ = tool.respectToLinkage_;

    name_ = tool.name_;
    frameType_ = tool.frameType_;
}


Tool::Tool(const Tool &tool)
    : Frame::Frame(tool.respectToFixed_, tool.name_, tool.id_, TOOL),
      respectToLinkage_(tool.respectToLinkage_)
{

}

Tool::Tool(TRANSFORM respectToFixed, string name, size_t id)
    : Frame::Frame(respectToFixed, name, id, TOOL),
      respectToLinkage_(respectToFixed)
{

}

// Tool Destructor
Tool::~Tool()
{
    
}

// Tool Methods
const TRANSFORM& Tool::respectToFixed() const { return respectToFixed_; }
void Tool::respectToFixed(TRANSFORM aCoordinate)
{
    respectToFixed_ = aCoordinate;
    if(hasLinkage)
        linkage_->updateFrames();
}

const TRANSFORM& Tool::respectToLinkage() const
{
    return respectToLinkage_;
}

TRANSFORM Tool::respectToRobot() const
{
    if(hasLinkage)
        return linkage_->respectToRobot_ * respectToLinkage_;
    else
        return respectToLinkage_;
}

TRANSFORM Tool::respectToWorld() const
{
    if(hasLinkage)
        return linkage_->respectToWorld() * respectToLinkage_;
    else
        return respectToLinkage_;
}

const Linkage* Tool::parentLinkage() const
{
    if(hasLinkage)
        return linkage_;
    else
        return NULL;
}

const Robot* Tool::parentRobot() const
{
    if(hasRobot)
        return robot_;
    else
        return NULL;
}

size_t Tool::getLinkageID()
{
    if(hasLinkage)
        return linkage_->id();
    else
        return 0;
}

string Tool::getLinkageName()
{
    if(hasLinkage)
        return linkage_->name();
    else
        return "";
}

size_t Tool::getRobotID()
{
    if(hasRobot)
        return robot_->id();
    else
        return 0;
}

string Tool::getRobotName()
{
    if(hasRobot)
        return robot_->name();
    else
        return "";
}

size_t Tool::getParentJointID()
{
    if(hasLinkage)
        if(linkage_->joints_.size() > 0)
            return linkage_->joints_.size()-1;

    return 0;
}

string Tool::getParentJointName()
{
    if(hasLinkage)
        if(linkage_->joints_.size() > 0)
            return linkage_->joints().back()->name();

    return "";
}

size_t Joint::getLinkageID()
{
    if(hasLinkage)
        return linkage_->id();
    else
        return 0;
}

string Joint::getLinkageName()
{
    if(hasLinkage)
        return linkage_->name();
    else
        return "";
}

size_t Joint::getRobotID()
{
    if(hasRobot)
        return robot_->id();
    else
        return 0;
}

string Joint::getRobotName()
{
    if(hasRobot)
        return robot_->name();
    else
        return "";
}

size_t Linkage::getParentLinkageID()
{
    if(hasParent)
        return parentLinkage_->id();
    else
        return 0;
}

string Linkage::getParentLinkageName()
{
    if(hasParent)
        return parentLinkage_->name();
    else
        return "";
}

size_t Linkage::getRobotID()
{
    if(hasRobot)
        return robot_->id();
    else
        return 0;
}

string Linkage::getRobotName()
{
    if(hasRobot)
        return robot_->name();
    else
        return "";
}

void Linkage::getChildIDs(vector<size_t> &ids)
{
    ids.resize(childLinkages_.size());
    for(size_t i=0; i<childLinkages_.size(); i++)
        ids.push_back(childLinkages_[i]->id());
}

void Linkage::getChildNames(vector<string> &names)
{
    names.resize(childLinkages_.size());
    for(size_t i=0; i<childLinkages_.size(); i++)
        names.push_back(childLinkages_[i]->name());
}

void Linkage::printChildren()
{
    cout << "Children of " << name() << ":" << endl;
    for(size_t i=0; i<childLinkages_.size(); i++)
        cout << childLinkages_[i]->name() << " ("
             << childLinkages_[i]->id() << ")" << endl;
}

void Tool::printInfo() const
{
    cout << frameTypeString() << " Info: " << name() << " (ID: " << id() << ")" << endl;
    cout << "Respect to fixed frame:" << endl;
    cout << respectToFixed().matrix() << endl << endl;
    cout << "Respect to linkage frame:" << endl;
    cout << respectToLinkage().matrix() << endl << endl;
    cout << "Respect to robot frame:" << endl;
    cout << respectToRobot().matrix() << endl << endl;
    cout << "Respect to world frame:" << endl;
    cout << respectToWorld().matrix() << endl << endl;
}

Tool Tool::Identity()
{
    Tool tool;
    return tool;
}


//------------------------------------------------------------------------------
// Linkage Lifecycle
//------------------------------------------------------------------------------
// Constructors

Linkage& Linkage::operator =( const Linkage& linkage )
{
    respectToFixed_ = linkage.respectToFixed_;
    respectToRobot_ = linkage.respectToRobot_;
    
    name_ = linkage.name_;
    id_ = linkage.id_;
    frameType_ = linkage.frameType_;
    
    joints_.resize(0);
    for(size_t i=0; i<linkage.joints_.size(); i++)
        addJoint(*(linkage.joints_[i]));
    setTool(linkage.tool_);
    
    updateFrames();
}

Linkage::Linkage(const Linkage &linkage)
    : Frame::Frame(linkage.respectToFixed_, linkage.name_,
                   linkage.id_, linkage.frameType_),
      respectToRobot_(linkage.respectToRobot_),
      tool_(linkage.tool_),
      initializing_(false),
      hasParent(false),
      hasChildren(false)
{
    for(size_t i=0; i<linkage.joints_.size(); i++)
        addJoint(*(linkage.joints_[i]));

    setTool(linkage.tool_);
    
    updateFrames();
}

Linkage::Linkage()
    : Frame::Frame(TRANSFORM::Identity(), "", 0, LINKAGE),
      respectToRobot_(TRANSFORM::Identity()),
      initializing_(false),
      hasParent(false),
      hasChildren(false)
{
    analyticalIK = Linkage::defaultAnalyticalIK;
}

Linkage::Linkage(TRANSFORM respectToFixed, string name, size_t id)
    : Frame::Frame(respectToFixed, name, id, LINKAGE),
      respectToRobot_(TRANSFORM::Identity()),
      initializing_(false),
      hasParent(false),
      hasChildren(false)
{
    analyticalIK = Linkage::defaultAnalyticalIK;
}


Linkage::Linkage(TRANSFORM respectToFixed, string name, size_t id, Joint joint, Tool tool)
    : Frame::Frame(respectToFixed, name, id, LINKAGE),
      respectToRobot_(respectToFixed),
      initializing_(false),
      hasParent(false),
      hasChildren(false)
{
    analyticalIK = Linkage::defaultAnalyticalIK;
    vector<Joint> joints(1);
    joints[0] = joint;
    initialize(joints, tool);
}

Linkage::Linkage(TRANSFORM respectToFixed, string name, size_t id, vector<Joint> joints, Tool tool)
    : Frame::Frame(respectToFixed, name, id, LINKAGE),
      respectToRobot_(respectToFixed),
      initializing_(false),
      hasParent(false),
      hasChildren(false)
{
    analyticalIK = Linkage::defaultAnalyticalIK;
    initialize(joints, tool);
}

// Destructor
Linkage::~Linkage()
{
    
}

//------------------------------------------------------------------------------
// Linkage Overloaded Operators
//------------------------------------------------------------------------------
// Assignment operator
const Linkage& Linkage::operator=(const VectorXd& someValues)
{
    values(someValues);
    return *this;
}

//--------------------------------------------------------------------------
// Linkage Public Member Functions
//--------------------------------------------------------------------------

Linkage* Linkage::parentLinkage() { return parentLinkage_; }

size_t Linkage::nChildren() const { return childLinkages_.size(); }

size_t Linkage::nJoints() const { return joints_.size(); }

const Joint& Linkage::const_joint(size_t jointIndex) const
{
    assert(jointIndex < nJoints());
    return *joints_[jointIndex];
}
const Joint& Linkage::const_joint(string jointName) const { return *joints_[jointNameToIndex_.at(jointName)]; }

Joint& Linkage::joint(size_t jointIndex)
{
    // FIXME: Remove assert
    assert(jointIndex < nJoints());
    return *joints_[jointIndex];
}
Joint& Linkage::joint(string jointName) { return *joints_[jointNameToIndex_.at(jointName)]; }

const vector<Joint*>& Linkage::const_joints() const { return joints_; }
vector<Joint*>& Linkage::joints() { return joints_; }


const Tool& Linkage::const_tool() const { return tool_; }
Tool& Linkage::tool() { return tool_; }


VectorXd Linkage::values() const
{
    VectorXd theValues(nJoints(),1);
    for (size_t i = 0; i < nJoints(); ++i) {
        theValues[i] = joints_[i]->value();
    }
    return theValues;
}

bool Linkage::values(const VectorXd& someValues)
{    
    if(someValues.size() == nJoints())
    {
        for (size_t i = 0; i < nJoints(); ++i) {
            joints_[i]->value(someValues(i));
        }
        updateFrames();
        return true;
    }
    
    std::cerr << "ERROR! Number of values (" << someValues.size() << ") does not match "
              << "the number of joints (" << nJoints() << ")!" << std::endl;
    return false;
}

const TRANSFORM& Linkage::respectToFixed() const { return respectToFixed_; }
void Linkage::respectToFixed(TRANSFORM aCoordinate)
{
    respectToFixed_ = aCoordinate;
    updateFrames();
}


const TRANSFORM& Linkage::respectToRobot() const
{
    return respectToRobot_;
}


TRANSFORM Linkage::respectToWorld() const
{
    if(hasRobot)
        return robot_->respectToWorld_ * respectToRobot_;
    else
        return TRANSFORM::Identity();
}

void Linkage::jacobian(MatrixXd& J, const vector<Joint*>& jointFrames, TRANSLATION location, const Frame* refFrame) const
{ // location should be specified respect to linkage coordinate frame
    
    size_t nCols = jointFrames.size();
    J.resize(6, nCols);
    
    TRANSLATION o_i, d_i, z_i; // Joint i location, offset, axis
    
    for (size_t i = 0; i < nCols; ++i) {
        
        o_i = jointFrames[i]->respectToLinkage_.translation(); // Joint i location
//        d_i = o_i - location; // Vector from location to joint i
        d_i = location - o_i; // Changing convention so that the position vector points away from the joint axis
//        z_i = jointFrames[i]->respectToLinkage_.rotation().col(2); // Joint i joint axis
        z_i = jointFrames[i]->respectToLinkage_.rotation()*jointFrames[i]->jointAxis_;

        // Set column i of Jocabian
        if (jointFrames[i]->jointType_ == REVOLUTE) {
//            J.block(0, i, 3, 1) = d_i.cross(z_i);
            J.block(0, i, 3, 1) = z_i.cross(d_i); // Changing convention to (w x r)
            J.block(3, i, 3, 1) = z_i;
        } else if(jointFrames[i]->jointType_ == PRISMATIC) {
            J.block(0, i, 3, 1) = z_i;
            J.block(3, i, 3, 1) = TRANSLATION::Zero();
        } else {
            J.block(0, i, 3, 1) = TRANSLATION::Zero();
            J.block(3, i, 3, 1) = TRANSLATION::Zero();
        }
        
    }
    
    // Jacobian transformation
    Matrix3d r(refFrame->respectToWorld().rotation().inverse() * respectToWorld().rotation());
    MatrixXd R(6,6);
    R << r, Matrix3d::Zero(), Matrix3d::Zero(), r;
    J = R * J;
}


void Linkage::printInfo() const
{
    cout << frameTypeString() << " Info: " << name() << " (ID: " << id() << ")" << endl;
    
    cout << "Respect to fixed frame:" << endl;
    cout << respectToFixed().matrix() << endl << endl;
    cout << "Respect to robot frame:" << endl;
    cout << respectToRobot().matrix() << endl << endl;
    cout << "Respect to world frame:" << endl;
    cout << respectToWorld().matrix() << endl << endl;
    
    cout << "Joints (ID, Name, Value): " << endl;
    for (vector<Joint*>::const_iterator jointIt = const_joints().begin();
         jointIt != const_joints().end(); ++jointIt) {
        cout << (*jointIt)->id() << ", " << (*jointIt)->name() << ", " << (*jointIt)->value() << endl;
    }
    
    for (vector<Joint*>::const_iterator jointIt = const_joints().begin();
         jointIt != const_joints().end(); ++jointIt) {
        (*jointIt)->printInfo();
    }
    const_tool().printInfo();
    
    MatrixXd J;
    jacobian(J, const_joints(), const_tool().respectToLinkage().translation(), this);
    
    cout << "Jacobian for " << name() << ":" << endl;
    cout << J.matrix() << endl;
    cout << "\n" << endl;
}

//------------------------------------------------------------------------------
// Linkage Private Member Functions
//------------------------------------------------------------------------------
void Linkage::initialize(vector<Joint> joints, Tool tool)
{
    initializing_ = true;
    
    for(size_t i = 0; i != joints.size(); ++i)
        addJoint(joints[i]);
    
    setTool(tool);
    
    initializing_ = false;

    updateFrames();
}

void Linkage::addJoint(Joint newJoint)
{
    Joint* tempJoint = new Joint(newJoint);
    size_t newIndex = joints_.size();
    joints_.push_back(tempJoint);
    joints_[newIndex]->id_ = newIndex;
    joints_[newIndex]->linkage_ = this;
    joints_[newIndex]->hasLinkage = true;
    jointNameToIndex_[joints_[newIndex]->name()] = newIndex;
    
    if(hasRobot)
    {
        joints_[newIndex]->robot_ = robot_;
        joints_[newIndex]->hasRobot = true;
        
        robot_->joints_.push_back(joints_[newIndex]);
        robot_->joints_.back()->id_ = joints_.size()-1;
        jointNameToIndex_[joints_.back()->name()] = joints_.size()-1;
    }
}

void Linkage::setTool(Tool newTool)
{
    tool_ = newTool;
    tool_.id_ = joints_.size();
    tool_.linkage_ = this;
    tool_.hasLinkage = true;

    if(hasRobot)
        tool_.Tool::robot_ = robot_;
    tool_.Tool::hasRobot = hasRobot;
}

void Linkage::setJointValue(size_t jointIndex, double val){joint(jointIndex).value(val);}

void Linkage::setJointValue(string jointName, double val){joint(jointName).value(val);}


void Linkage::updateFrames()
{
    if (~initializing_) {
        for (size_t i = 0; i < joints_.size(); ++i) {
            if (i == 0) {
                joints_[i]->respectToLinkage_ = joints_[i]->respectToFixedTransformed_;
                
            } else {
                joints_[i]->respectToLinkage_ = joints_[i-1]->respectToLinkage_ * joints_[i]->respectToFixedTransformed_;
            }
        }
        if(joints_.size() > 0)
            tool_.respectToLinkage_ = joints_[joints_.size()-1]->respectToLinkage_ * tool_.respectToFixed_;
        else
            tool_.respectToLinkage_ = tool_.respectToFixed_;
        
        if(hasChildren)
            updateChildLinkage();
    }
}


void Linkage::updateChildLinkage()
{
    for (size_t i = 0; i < nChildren(); ++i) {
        childLinkages_[i]->respectToRobot_ = tool_.respectToRobot() * childLinkages_[i]->respectToFixed_;
        if(childLinkages_[i]->hasChildren)
            childLinkages_[i]->updateChildLinkage();
    }
}

bool Linkage::defaultAnalyticalIK(VectorXd& q, const TRANSFORM& B, const VectorXd& qPrev) {
    // This function is just a place holder and should not be used. The analyticalIK function pointer should be set to the real analytical IK function.
    q = NAN * qPrev;
    return false;
}






