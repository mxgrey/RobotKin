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


namespace RobotKin {

    typedef Eigen::Isometry3d TRANSFORM;
    typedef Eigen::Vector3d   TRANSLATION;
    // TODO: Consider making these classes that inherit their respective EigenC++ types
    // That way I can make .respectTo() and other such useful member functions

    typedef Eigen::Vector3d   AXIS;
    
    class Robot;
    class Linkage;
    class Joint;
    class Tool;
    
    //------------------------------------------------------------------------------
    // Typedefs
    //------------------------------------------------------------------------------
    
    enum FrameType {
        UNKNOWN,
        JOINT,
        TOOL,
        LINKAGE,
        ROBOT,
        WORLD,

        FRAME_TYPE_SIZE
    };
    
    static const char *FrameType_string[FRAME_TYPE_SIZE] =
    {
        "UNKNOWN",
        "JOINT",
        "TOOL",
        "LINKAGE",
        "ROBOT",
        "WORLD"
    };

    std::string FrameType_to_string(FrameType type);
    

    typedef enum {
        RK_SOLVED = 0,
        RK_DIVERGED,
        RK_CONVERGED,
        RK_NO_SOLUTION,
        RK_INVALID_JOINT,
        RK_INVALID_LINKAGE,
        RK_INVALID_FRAME_TYPE,

        RK_SOLVER_NOT_READY,


        RK_TYPE_SIZE
    } rk_result_t;

    static const char *rk_result_string[RK_TYPE_SIZE] =
    {
        "RK_SOLVED",
        "RK_DIVERGED",
        "RK_CONVERGED", // TODO: Is this really any different than RK_SOLVED?
        "RK_NO_SOLUTION",
        "RK_INVALID_JOINT",
        "RK_INVALID_LINKAGE",
        "RK_INVALID_FRAME_TYPE"

        "RK_SOLVER_NOT_READY"
    };

    std::string rk_result_to_string(rk_result_t result);

    
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

        std::string name() const;
        void name(std::string newName);
        
        FrameType frameType() const;
        std::string frameTypeString() const;
        
        virtual const TRANSFORM& respectToFixed() const = 0;
        virtual void respectToFixed(TRANSFORM aCoordinate) = 0;
        
        virtual TRANSFORM respectToWorld() const = 0;

        TRANSFORM respectTo(const Frame* aFrame) const;
        TRANSFORM withRespectTo(const Frame &frame) const;
        TRANSFORM withRespectTo(const Robot &robot) const;
        TRANSFORM withRespectTo(const Linkage &linkage) const;
        TRANSFORM withRespectTo(const Joint &joint) const;
        TRANSFORM withRespectTo(const Tool &tool) const;
        
        virtual void printInfo() const;
        
        
    protected:
        //--------------------------------------------------------------------------
        // Frame Constructor
        //--------------------------------------------------------------------------
        Frame(TRANSFORM respectToFixed = TRANSFORM::Identity(),
              std::string name = "",
              size_t id = 0,
              FrameType frameType = UNKNOWN);
        
        //--------------------------------------------------------------------------
        // Frame Protected Member Variables
        //--------------------------------------------------------------------------
        std::string name_;
        size_t id_;
        FrameType frameType_;
        TRANSFORM respectToFixed_; // Coordinates with respect to some fixed frame in nominal position

        Robot* robot_;
        Linkage* linkage_;

        bool hasRobot;
        bool hasLinkage;
        
    }; // class Frame
    
} // namespace RobotKin

#endif


