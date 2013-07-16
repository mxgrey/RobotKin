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
#include "Robot.h"



//------------------------------------------------------------------------------
// Namespaces
//------------------------------------------------------------------------------
using namespace RobotKin;
using namespace std;
using namespace Eigen;

//------------------------------------------------------------------------------
// Lifecycle
//------------------------------------------------------------------------------
// Constructors
Frame::Frame(TRANSFORM respectToFixed, string name, size_t id, FrameType frameType)
    : name_(name),
      id_(id),
      frameType_(frameType),
      respectToFixed_(respectToFixed),
      linkage_(NULL),
      robot_(NULL),
      hasRobot(false),
      hasLinkage(false)
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

void Frame::name(string newName)
{
    if(hasRobot)
    {
        if(frameType()==LINKAGE)
        {
            robot_->linkageNameToIndex_[newName] = robot_->linkageNameToIndex_[name_];
            robot_->linkageNameToIndex_.erase(name_);
        }

        if(frameType()==JOINT)
        {
            robot_->jointNameToIndex_[newName] = robot_->jointNameToIndex_[name_];
            robot_->jointNameToIndex_.erase(name_);
        }
    }
    name_ = newName;
}

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

TRANSFORM Frame::respectTo(const Frame* aFrame) const
{
    return aFrame->respectToWorld().inverse() * respectToWorld();
}

TRANSFORM Frame::withRespectTo(const Frame &frame) const { return respectTo(&frame); }
TRANSFORM Frame::withRespectTo(const Robot &robot) const { return respectTo(&robot); }
TRANSFORM Frame::withRespectTo(const Linkage &linkage) const { return respectTo(&linkage); }
TRANSFORM Frame::withRespectTo(const Joint &joint) const { return respectTo(&joint); }

void Frame::printInfo() const
{
    cout << frameTypeString() << " Frame Info: " << name() << " (ID: " << id() << ")" << endl;
    cout << "Respect to fixed frame:" << endl;
    cout << respectToFixed().matrix() << endl << endl;
    cout << "Respect to world frame:" << endl;
    cout << respectToWorld().matrix() << endl<< endl;
}



