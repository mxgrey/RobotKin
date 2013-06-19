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
using namespace std;
using namespace Eigen;

namespace RobotKin {
    
    
    //------------------------------------------------------------------------------
    // Typedefs and Enums
    //------------------------------------------------------------------------------
    typedef Matrix<double, 6, Dynamic> Matrix6Xd;
    
    enum JointType {
        ANCHOR = 0,
        REVOLUTE,
        PRISMATIC
    };
    
    
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
        // Linkage Nested Classes
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
            Link(double mass, Vector3d com, string name = ""); // TODO
            Link(double mass, Vector3d com, Matrix3d inertiaTensor, string name = ""); // TODO

            void setMass(double mass, Vector3d com); // TODO
            void setInertiaTensor(Matrix3d inertiaTensor); // TODO

        protected:
            bool massProvided;
            bool tensorProvided;

            double mass_;
            Vector3d com_;
            Matrix3d tensor_;

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
            
            //----------------------------------------------------------------------
            // Joint Lifecycle
            //----------------------------------------------------------------------
            // Constructors
            Joint(Isometry3d respectToFixed = Isometry3d::Identity(),
                  string name = "",
                  size_t id = 0,
                  JointType jointType = REVOLUTE,
                  double minValue = -M_PI,
                  double maxValue = M_PI);
            Joint(const Joint& joint);
            
            // Destructor
            virtual ~Joint();
            
            //----------------------------------------------------------------------
            // Joint Overloaded Operators
            //----------------------------------------------------------------------
            // Assignment operator
            const Linkage::Joint& operator=(const double value);
            Linkage::Joint& operator=(const Linkage::Joint& joint); // TODO: Test this
            
            //----------------------------------------------------------------------
            // Joint Public Member Functions
            //----------------------------------------------------------------------
            double value() const;
            void value(double value);
            
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
            Isometry3d respectToFixedTransformed_; // Coordinates transformed according to the joint value and type with respect to respectToFixed frame
            Isometry3d respectToLinkage_; // Coordinates with respect to linkage base frame
            Linkage* linkage_;
            Robot* robot_;
            
            bool hasLinkage;
            bool hasRobot;
            
        }; // class Joint
        
        class Tool : public Frame
        {
            
            //----------------------------------------------------------------------
            // Tool Friends
            //----------------------------------------------------------------------
            friend class Linkage;
            friend class Robot;
            
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
            
            static Linkage::Tool Identity();

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
            Linkage* linkage_;
            Robot* robot_;
            
            bool hasRobot;
            bool hasLinkage;
            
        }; // class Tool
        
        
        //--------------------------------------------------------------------------
        // Linkage Lifecycle
        //--------------------------------------------------------------------------
        // Constructors
        Linkage();
        Linkage(const Linkage& linkage);
        Linkage(Isometry3d respectToFixed,
                string name, size_t id,
                Linkage::Joint joint,
                Linkage::Tool tool = Linkage::Tool::Identity());
        Linkage(Isometry3d respectToFixed,
                string name,
                size_t id,
                vector<Linkage::Joint> joints,
                Linkage::Tool tool = Linkage::Tool::Identity());
        
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
        
        
        size_t nJoints() const;
        const Linkage::Joint& const_joint(size_t jointIndex) const;
        const Linkage::Joint& const_joint(string jointName) const;
        
        Linkage::Joint& joint(size_t jointIndex);
        Linkage::Joint& joint(string jointName);
        
        const vector<Linkage::Joint*>& const_joints() const;
        vector<Linkage::Joint*>& joints();

        void addJoint(Linkage::Joint newJoint); // TODO
        void insertJoint(Linkage::Joint newJoint, size_t jointIndex); // TODO

        void setTool(Linkage::Tool newTool);        
//        void addTool(Linkage::Tool newTool); // TODO
//        void chooseTool(size_t toolIndex); // TODO
//        void chooseTool(string toolName); // TODO
        
        const Linkage::Tool& const_tool() const;
        Linkage::Tool& tool();
        
        VectorXd values() const;
        bool values(const VectorXd &someValues);
        
        const Isometry3d& respectToFixed() const;
        void respectToFixed(Isometry3d aCoordinate);
        
        const Isometry3d& respectToRobot() const;
        
        Isometry3d respectToWorld() const;
        
        void jacobian(MatrixXd& J, const vector<Linkage::Joint*>& jointFrames, Vector3d location, const Frame* refFrame) const;
        
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
        Robot* robot_;
        Linkage* parentLinkage_;
        vector<Linkage*> childLinkages_;
        vector<Linkage::Joint*> joints_;
        Linkage::Tool tool_;
        // TODO: Consider allowing multiple switchable tools
        //vector<Linkage::Tool> tools_;
//        size_t activeTool_;
        
        bool hasRobot;
        bool hasParent;
        bool hasChildren;
        
    private:
        //--------------------------------------------------------------------------
        // Linkage Public Member Functions
        //--------------------------------------------------------------------------
        void initialize(vector<Linkage::Joint> joints, Linkage::Tool tool);
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


