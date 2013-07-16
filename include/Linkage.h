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
                  std::string name = "",
                  size_t id = 0,
                  JointType jointType = REVOLUTE,
                  Eigen::Vector3d axis=Eigen::Vector3d::UnitZ(),
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

            void setJointAxis(Eigen::Vector3d axis);
            
            const Eigen::Isometry3d& respectToFixed() const;
            void respectToFixed(Eigen::Isometry3d aCoordinate);
            
            const Eigen::Isometry3d& respectToFixedTransformed() const;
            
            const Eigen::Isometry3d& respectToLinkage() const;
            
            Eigen::Isometry3d respectToRobot() const;
            
            Eigen::Isometry3d respectToWorld() const;

            size_t getLinkageID();
            std::string getLinkageName();

            size_t getRobotID();
            std::string getRobotName();

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
            Eigen::Vector3d jointAxis_;
            Eigen::Isometry3d respectToFixedTransformed_; // Coordinates transformed according to the joint value and type with respect to respectToFixed frame
            Eigen::Isometry3d respectToLinkage_; // Coordinates with respect to linkage base frame


            
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
                 std::string name = "",
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
            std::string getRobotName();

            size_t getLinkageID();
            std::string getLinkageName();

            size_t getParentJointID();
            std::string getParentJointName();


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
        Linkage(Eigen::Isometry3d respectToFixed, std::string name, size_t id);
        Linkage(Eigen::Isometry3d respectToFixed,
                std::string name, size_t id,
                Linkage::Joint joint,
                Linkage::Tool tool = Linkage::Tool::Identity());
        Linkage(Eigen::Isometry3d respectToFixed,
                std::string name,
                size_t id,
                std::vector<Linkage::Joint> joints,
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
        void setJointValue(std::string jointName, double val);
        
        size_t nJoints() const;
        const Linkage::Joint& const_joint(size_t jointIndex) const;
        const Linkage::Joint& const_joint(std::string jointName) const;
        
        Linkage::Joint& joint(size_t jointIndex);
        Linkage::Joint& joint(std::string jointName);
        
        const std::vector<Linkage::Joint*>& const_joints() const;
        std::vector<Linkage::Joint*>& joints();

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
        
        const Eigen::Isometry3d& respectToFixed() const;
        void respectToFixed(Eigen::Isometry3d aCoordinate);
        
        const Eigen::Isometry3d& respectToRobot() const;
        
        Eigen::Isometry3d respectToWorld() const;
        
        void jacobian(Eigen::MatrixXd& J, const std::vector<Linkage::Joint*>& jointFrames, Eigen::Vector3d location, const Frame* refFrame) const;
        
        void printInfo() const;
        
        size_t getParentLinkageID();
        std::string getParentLinkageName();
        
        size_t getRobotID();
        std::string getRobotName();
        
        void getChildIDs(std::vector<size_t>& ids);
        void getChildNames(std::vector<std::string>& names);
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
        std::vector<Linkage*> childLinkages_;
        std::vector<Linkage::Joint*> joints_;
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
        void initialize(std::vector<Linkage::Joint> joints, Linkage::Tool tool);
        void updateFrames();
        void updateChildLinkage();
        static bool defaultAnalyticalIK(Eigen::VectorXd& q, const Eigen::Isometry3d& B, const Eigen::VectorXd& qPrev);
        
        
        //--------------------------------------------------------------------------
        // Linkage Private Member Variables
        //--------------------------------------------------------------------------
        bool initializing_;
        std::map<std::string, size_t> jointNameToIndex_;
        
        
    }; // class Linkage

    
} // namespace RobotKin

#endif


