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
#include <math.h>
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

namespace RobotKin
{
    
    
    //------------------------------------------------------------------------------
    // Typedefs and Enums
    //------------------------------------------------------------------------------
    typedef Eigen::Matrix<double, 6, Eigen::Dynamic> Matrix6Xd;
    
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
            Link(double mass, Eigen::Vector3d com, string name = "");
            Link(double mass, Eigen::Vector3d com, Eigen::Matrix3d inertiaTensor, string name = "");

            // Destructor
            virtual ~Link();

            //----------------------------------------------------------------------
            // Link Public Member Functions
            //----------------------------------------------------------------------
            string name() const;
            void name(string name);

            double mass() const;
            void mass(double mass);

            Eigen::Vector3d com() const;
            void com(Eigen::Vector3d com);
            
            Eigen::Matrix3d inertiaTensor() const;
            void inertiaTensor(Eigen::Matrix3d inertiaTensor);

        protected:
            bool massProvided;
            bool tensorProvided;

            string name_;
            double mass_;
            Eigen::Vector3d com_;
            Eigen::Matrix3d inertiaTensor_;

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
            Joint(Eigen::Isometry3d respectToFixed = Eigen::Isometry3d::Identity(),
                  string name = "",
                  size_t id = 0,
                  JointType jointType = REVOLUTE,
                  Eigen::Vector3d axis=Eigen::Vector3d::UnitZ(),
                  double minValue = -M_PI,
                  double maxValue = M_PI,
                  double minVel = -INFINITY,
                  double maxVel = INFINITY,
                  double minAcc = -INFINITY,
                  double maxAcc = INFINITY);
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

            double max() const;
            void max(double max);

            double min() const;

            void min(double min);

            double vel() const;
            void vel(double vel);            

            double maxVel() const;
            void maxVel(double maxVel);

            double minVel() const;
            void minVel(double minVel);

            double acc() const;
            void acc(double acc);            

            double maxAcc() const;
            void maxAcc(double maxAcc);

            double minAcc() const;
            void minAcc(double minAcc);

            void setJointAxis(Eigen::Vector3d axis);

            const Linkage::Link& const_link() const;
            Linkage::Link& link();
            
            const Eigen::Isometry3d& respectToFixed() const;
            void respectToFixed(Eigen::Isometry3d aCoordinate);
            
            const Eigen::Isometry3d& respectToFixedTransformed() const;
            
            const Eigen::Isometry3d& respectToLinkage() const;
            
            Eigen::Isometry3d respectToRobot() const;
            
            Eigen::Isometry3d respectToWorld() const;

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
            double vel_; // Current joint velocity
            double acc_; // Current joint acceleration

            const Linkage* linkage() const;

            const Robot* parentRobot() const;
            
        private:
            //----------------------------------------------------------------------
            // Joint Private Member Variables
            //----------------------------------------------------------------------
            JointType jointType_; // Type of joint (REVOLUTE or PRISMATIC)
            double min_; // Minimum joint value
            double max_; // Maximum joint value
            double minVel_; // Minimum joint velocity
            double maxVel_; // Maximum joint velocity
            double minAcc_; // Minimum joint acceleration
            double maxAcc_; // Maximum joint acceleration
            Eigen::Vector3d jointAxis_;
            Eigen::Isometry3d respectToFixedTransformed_; // Coordinates transformed according to the joint value and type with respect to respectToFixed frame
            Eigen::Isometry3d respectToLinkage_; // Coordinates with respect to linkage base frame
            Link link_;


            
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
            Tool(Eigen::Isometry3d respectToFixed = Eigen::Isometry3d::Identity(),
                 string name = "",
                 size_t id = 0);
            
            // Destructor
            virtual ~Tool();
            
            //----------------------------------------------------------------------
            // Tool Public Member Functions
            //----------------------------------------------------------------------
            Tool& operator =(const Tool& tool);

            const Eigen::Isometry3d& respectToFixed() const;
            void respectToFixed(Eigen::Isometry3d aCoordinate);
            
            const Eigen::Isometry3d& respectToLinkage() const;
            
            Eigen::Isometry3d respectToRobot() const;
            
            Eigen::Isometry3d respectToWorld() const;
            
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
            Eigen::Isometry3d respectToLinkage_; // Coordinates with respect to linkage base frame
            
        }; // class Tool
        
        
        //--------------------------------------------------------------------------
        // Linkage Lifecycle
        //--------------------------------------------------------------------------
        // Constructors
        Linkage();
        Linkage(const Linkage& linkage);
        Linkage(Eigen::Isometry3d respectToFixed, string name, size_t id);
        Linkage(Eigen::Isometry3d respectToFixed,
                string name, size_t id,
                Linkage::Joint joint,
                Linkage::Tool tool = Linkage::Tool::Identity());
        Linkage(Eigen::Isometry3d respectToFixed,
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
        const Linkage& operator=(const Eigen::VectorXd& values);
        
        
        //--------------------------------------------------------------------------
        // Linkage Public Member Functions
        //--------------------------------------------------------------------------
        
        Linkage* parentLinkage();        
        
        size_t nChildren() const;
        
        void setJointValue(size_t jointIndex, double val);
        void setJointValue(string jointName, double val);
        
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
        
        Eigen::VectorXd values() const;
        bool values(const Eigen::VectorXd &someValues);

        Eigen::VectorXd minValues() const;
        Eigen::VectorXd maxValues() const;

        Eigen::VectorXd vels() const;
        void vels(const Eigen::VectorXd &someVels);

        Eigen::VectorXd minVels() const;
        Eigen::VectorXd maxVels() const;

        Eigen::VectorXd accs() const;
        void accs(const Eigen::VectorXd &someAccs);

        Eigen::VectorXd minAccs() const;
        Eigen::VectorXd maxAccs() const;
        
        const Eigen::Isometry3d& respectToFixed() const;
        void respectToFixed(Eigen::Isometry3d aCoordinate);
        
        const Eigen::Isometry3d& respectToRobot() const;
        
        Eigen::Isometry3d respectToWorld() const;
        
        void jacobian(Eigen::MatrixXd& J, const vector<Linkage::Joint*>& jointFrames, Eigen::Vector3d location, const Frame* refFrame) const;
        
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
        bool (*analyticalIK)(Eigen::VectorXd& q, const Eigen::Isometry3d& B, const Eigen::VectorXd& qPrev);
        
    protected:
        //--------------------------------------------------------------------------
        // Linkage Protected Member Variables
        //--------------------------------------------------------------------------
        
        Eigen::Isometry3d respectToRobot_; // Coordinates with respect to robot base frame
        Linkage* parentLinkage_;
        vector<Linkage*> childLinkages_;
        vector<Linkage::Joint*> joints_;
        Linkage::Tool tool_;
        // TODO: Consider allowing multiple switchable tools
        //vector<Linkage::Tool> tools_;
//        size_t activeTool_;

        bool hasParent;
        bool hasChildren;
        
    private:
        //--------------------------------------------------------------------------
        // Linkage Public Member Functions
        //--------------------------------------------------------------------------
        void initialize(vector<Linkage::Joint> joints, Linkage::Tool tool);
        void updateFrames();
        void updateChildLinkage();
        static bool defaultAnalyticalIK(Eigen::VectorXd& q, const Eigen::Isometry3d& B, const Eigen::VectorXd& qPrev);
        
        
        //--------------------------------------------------------------------------
        // Linkage Private Member Variables
        //--------------------------------------------------------------------------
        bool initializing_;
        map<string, size_t> jointNameToIndex_;
        
        
    }; // class Linkage

    
} // namespace RobotKin

#endif


