/*
 -------------------------------------------------------------------------------
 Linkage.h
 Robot Library Project
 
 CLASS NAME:
 Linkage
 
 DESCRIPTION:
 description...
 
 FILES:
 Linkage.h
 Linkage.cpp
 
 DEPENDENCIES:
 
 
 CONSTRUCTORS:
 Linkage();
 
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



#ifndef _Linkage_h_ 
#define _Linkage_h_



//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include "Frame.h"
#include <string>
#include <vector>
#include <map>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

//------------------------------------------------------------------------------
// Class Declarations
//------------------------------------------------------------------------------
namespace RobotKin
{
    class Robot;
}

//------------------------------------------------------------------------------
// Namespaces
//------------------------------------------------------------------------------

namespace RobotKin {
    
    //------------------------------------------------------------------------------
    // Typedefs and Enums
    //------------------------------------------------------------------------------
    typedef Eigen::Matrix<double, 6, Eigen::Dynamic> Matrix6Xd;
    
    enum JointType {
        ANCHOR = 0,
        REVOLUTE,
        PRISMATIC
    };
    
    class Joint : public Frame
    {

        //----------------------------------------------------------------------
        // Joint Friends
        //----------------------------------------------------------------------
        friend class Linkage;
        friend class Robot;
        friend class Link;

    public:
        //--------------------------------------------------------------------------
        // Link Nested Class
        //--------------------------------------------------------------------------
        class Link
        {
            friend class Linkage;
            friend class Joint;

        public:
            //----------------------------------------------------------------------
            // Link Lifecycle
            //----------------------------------------------------------------------
            // Constructors
            Link(); // TODO
            Link(double mass, Eigen::Vector3d com, std::string name = ""); // TODO
            Link(double mass, Eigen::Vector3d com, Eigen::Matrix3d inertiaTensor, std::string name = ""); // TODO

            void setMass(double mass, Eigen::Vector3d com); // TODO
            void setInertiaTensor(Eigen::Matrix3d inertiaTensor); // TODO

        protected:
            bool massProvided;
            bool tensorProvided;

            double mass_;
            Eigen::Vector3d com_;
            Eigen::Matrix3d tensor_;

        }; // Class Link


        //----------------------------------------------------------------------
        // Joint Lifecycle
        //----------------------------------------------------------------------
        // Constructors
        Joint(Isometry3d respectToFixed = Isometry3d::Identity(),
              string name = "",
              size_t id = 0,
              JointType jointType = REVOLUTE,
              Vector3d axis=Eigen::Vector3d::UnitZ(),
              double minValue = -M_PI,
              double maxValue = M_PI);
        Joint(const Joint& joint);

        // Destructor
        virtual ~Joint();

        //----------------------------------------------------------------------
        // Joint Overloaded Operators
        //----------------------------------------------------------------------
        // Assignment operator
        const Joint& operator=(const double value);
        Joint& operator=(const Joint& joint); // TODO: Test this

        //----------------------------------------------------------------------
        // Joint Public Member Functions
        //----------------------------------------------------------------------
        double value() const;
        void value(double newValue);

        double min() const;
        double max() const;
        void min(double newMin);
        void max(double newMax);

        void setJointAxis(Eigen::Vector3d axis);

        const Isometry3d& respectToFixed() const;
        void respectToFixed(Isometry3d aCoordinate);

        const Isometry3d& respectToFixedTransformed() const;

        const Isometry3d& respectToLinkage() const;

        Isometry3d respectToRobot() const;

        Isometry3d respectToWorld() const;

        size_t getLinkageID();
        string getLinkageName();

        size_t getRobotID();
        string getRobotName();

//            size_t getParentJointID();
//            string getParentJointName();

        void printInfo() const;

    protected:
        //----------------------------------------------------------------------
        // Joint Protected Member Variables
        //----------------------------------------------------------------------
        double value_; // Current joint value (R type = joint angle, P type = joint length)

        const Linkage* linkage() const;

        const Robot* parentRobot() const;

    private:
        //----------------------------------------------------------------------
        // Joint Private Member Variables
        //----------------------------------------------------------------------
        JointType jointType_; // Type of joint (REVOLUTE or PRISMATIC)
        double min_; // Minimum joint value
        double max_; // Maximum joint value
        Vector3d jointAxis_;
        Isometry3d respectToFixedTransformed_; // Coordinates transformed according to the joint value and type with respect to respectToFixed frame
        Isometry3d respectToLinkage_; // Coordinates with respect to linkage base frame


    }; // class Joint

    class Tool : public Frame
    {

        //----------------------------------------------------------------------
        // Tool Friends
        //----------------------------------------------------------------------
        friend class Linkage;
        friend class Robot;
        friend class Frame;

    public:
        //----------------------------------------------------------------------
        // Tool Lifecycle
        //----------------------------------------------------------------------
        // Constructors
        Tool(const Tool& tool);
        Tool(Isometry3d respectToFixed = Isometry3d::Identity(),
             string name = "",
             size_t id = 0);

        // Destructor
        virtual ~Tool();

        //----------------------------------------------------------------------
        // Tool Public Member Functions
        //----------------------------------------------------------------------
        Tool& operator =(const Tool& tool);

        const Isometry3d& respectToFixed() const;
        void respectToFixed(Isometry3d aCoordinate);

        const Isometry3d& respectToLinkage() const;

        Isometry3d respectToRobot() const;

        Isometry3d respectToWorld() const;

        void printInfo() const;

        static Tool Identity();

        size_t getRobotID();
        string getRobotName();

        size_t getLinkageID();
        string getLinkageName();

        size_t getParentJointID();
        string getParentJointName();


    protected:

        const Linkage* parentLinkage() const;

        const Robot* parentRobot() const;


    private:
        //----------------------------------------------------------------------
        // Tool Private Member Variables
        //----------------------------------------------------------------------
        Isometry3d respectToLinkage_; // Coordinates with respect to linkage base frame

    }; // class Tool

    
    class Linkage : public Frame
    {
        //--------------------------------------------------------------------------
        // Linkage Friends
        //--------------------------------------------------------------------------
        friend class Joint;
        friend class Tool;
        friend class Robot;
        
    public:


        
        
        //--------------------------------------------------------------------------
        // Linkage Lifecycle
        //--------------------------------------------------------------------------
        // Constructors
        Linkage();
        Linkage(const Linkage& linkage);
        Linkage(Isometry3d respectToFixed, string name, size_t id);
        Linkage(Isometry3d respectToFixed,
                string name, size_t id,
                Joint joint,
                Tool tool = Tool::Identity());
        Linkage(Isometry3d respectToFixed,
                string name,
                size_t id,
                vector<Joint> joints,
                Tool tool = Tool::Identity());
        
        // Destructor
        virtual ~Linkage();
        
        
        //--------------------------------------------------------------------------
        // Linkage Overloaded Operators
        //--------------------------------------------------------------------------
        // Assignment operator
        Linkage& operator =( const Linkage& linkage );
        const Linkage& operator=(const VectorXd& values);
        
        
        //--------------------------------------------------------------------------
        // Linkage Public Member Functions
        //--------------------------------------------------------------------------
        
        Linkage* parentLinkage();        
        
        size_t nChildren() const;
        
        void setJointValue(size_t jointIndex, double val);
        void setJointValue(string jointName, double val);
        
        size_t nJoints() const;
        const Joint& const_joint(size_t jointIndex) const;
        const Joint& const_joint(string jointName) const;
        
        Joint& joint(size_t jointIndex);
        Joint& joint(string jointName);
        
        const vector<Joint*>& const_joints() const;
        vector<Joint*>& joints();

        void addJoint(Joint newJoint); // TODO
        void insertJoint(Joint newJoint, size_t jointIndex); // TODO

        void setTool(Tool newTool);
//        void addTool(Linkage::Tool newTool); // TODO
//        void chooseTool(size_t toolIndex); // TODO
//        void chooseTool(string toolName); // TODO
        
        const Tool& const_tool() const;
        Tool& tool();
        
        VectorXd values() const;
        bool values(const VectorXd &someValues);
        
        const Isometry3d& respectToFixed() const;
        void respectToFixed(Isometry3d aCoordinate);
        
        const Isometry3d& respectToRobot() const;
        
        Isometry3d respectToWorld() const;
        
        void jacobian(MatrixXd& J, const vector<Joint*>& jointFrames, Vector3d location, const Frame* refFrame) const;
        
        void printInfo() const;
        
        size_t getParentLinkageID();
        string getParentLinkageName();
        
        size_t getRobotID();
        string getRobotName();
        
        void getChildIDs(vector<size_t>& ids);
        void getChildNames(vector<string>& names);
        void printChildren();
        
        //--------------------------------------------------------------------------
        // Linkage Public Member Variables
        //--------------------------------------------------------------------------
        bool (*analyticalIK)(VectorXd& q, const Isometry3d& B, const VectorXd& qPrev);
        
    protected:
        //--------------------------------------------------------------------------
        // Linkage Protected Member Variables
        //--------------------------------------------------------------------------
        
        Isometry3d respectToRobot_; // Coordinates with respect to robot base frame
        Linkage* parentLinkage_;
        vector<Linkage*> childLinkages_;
        vector<Joint*> joints_;
        Tool tool_;
        // TODO: Consider allowing multiple switchable tools
        //vector<Linkage::Tool> tools_;
//        size_t activeTool_;

        bool hasParent;
        bool hasChildren;
        
    private:
        //--------------------------------------------------------------------------
        // Linkage Public Member Functions
        //--------------------------------------------------------------------------
        void initialize(vector<Joint> joints, Tool tool);
        void updateFrames();
        void updateChildLinkage();
        static bool defaultAnalyticalIK(VectorXd& q, const Isometry3d& B, const VectorXd& qPrev);
        
        
        //--------------------------------------------------------------------------
        // Linkage Private Member Variables
        //--------------------------------------------------------------------------
        bool initializing_;
        map<string, size_t> jointNameToIndex_;
        
        
    }; // class Linkage

    
} // namespace RobotKin

#endif


