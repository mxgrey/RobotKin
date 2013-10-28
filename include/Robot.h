/*
 -------------------------------------------------------------------------------
 Robot.h
 robotTest Project
 
 CLASS NAME:
 Robot
 
 DESCRIPTION:
 description...
 
 FILES:
 Robot.h
 Robot.cpp
 
 DEPENDENCIES:
 
 
 CONSTRUCTORS:
 Robot();
 
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
 1.0 - 5/15/13 - Rowland O'Flaherty ( rowlandoflaherty.com )
 
 -------------------------------------------------------------------------------
 */



#ifndef _Robot_h_
#define _Robot_h_



//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include "Frame.h"
#include "Linkage.h"
#include <vector>
#include <map>
#include <string>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "Constraints.h"



// TODO: Add a user-specified level of assertiveness

namespace RobotKin {


    //------------------------------------------------------------------------------
    // Typedefs
    //------------------------------------------------------------------------------

	// Sort parentIndices and linkages
    struct indexParentIndexPair {
        size_t I;
        int pI;
    
        bool operator<( const indexParentIndexPair& rhs ) const {
            return pI < rhs.pI;
        }
    };

    
    
    class Robot : public Frame
    {
        //--------------------------------------------------------------------------
        // Robot Friends
        //--------------------------------------------------------------------------
        friend class Linkage;
        friend class Frame;
        
    public:
        //--------------------------------------------------------------------------
        // Robot Lifecycle
        //--------------------------------------------------------------------------
        // Constructors
        Robot();
        Robot(std::vector<Linkage> linkageObjs, std::vector<int> parentIndices);

        Robot(std::string filename, std::string name="", size_t id=0);
        bool loadURDF(std::string filename);
        bool loadURDFString(std::string filename);
        
        // Destructor
        virtual ~Robot();

        bool imposeLimits;

        bool verbose;
        
        //--------------------------------------------------------------------------
        // Robot Public Member Functions
        //--------------------------------------------------------------------------
        size_t nLinkages() const;
        
        size_t linkageIndex(std::string linkageName) const;

        rk_result_t jointNamesToIndices(const std::vector<std::string> &jointNames,
                                        IntArray &jointIndices);

        rk_result_t linkageNamesToIndices(const std::vector<std::string> &linkageNames,
                                          IntArray &linkageIndices);
        
        // Getting individual linkages
        const Linkage& const_linkage(size_t linkageIndex) const;
        const Linkage& const_linkage(std::string linkageName) const;
        
        Linkage& linkage(size_t linkageIndex);
        Linkage& linkage(std::string linkageName);
        
        // Getting all the linkages
        const std::vector<Linkage*>& const_linkages() const;
        std::vector<Linkage*>& linkages();
        
        // Adding new linkages
        void addLinkage(int parentIndex, std::string name);
        void addLinkage(std::string parentName, std::string name);
        void addLinkage(Linkage linkage, std::string parentName, std::string name);
        void addLinkage(Linkage linkage, int parentIndex, std::string name);
        
        // TODO: Mention in documentation that this function is faster if "from" is the anchor point
        IntArray jointChain(std::string from, std::string to);
        IntArray jointChain(size_t from, size_t to);
        
        // Getting joint information
        size_t nJoints() const;
        size_t jointIndex(std::string jointName) const;

        const Joint& const_joint(size_t jointIndex) const;
        const Joint& const_joint(std::string jointName) const;

        Joint& joint(size_t jointIndex);
        Joint& joint(std::string jointName);

        // Convenience function
        rk_result_t setJointValue(size_t jointIndex, double val, bool update=true);
        rk_result_t setJointValue(std::string jointName, double val, bool update=true);
        
        const std::vector<Joint*>& const_joints() const;
        std::vector<Joint*>& joints();
        
        Eigen::VectorXd values() const;
        void values(const Eigen::VectorXd& allValues);
        void values(const IntArray &jointIndices, const Eigen::VectorXd& jointValues);
        
        const TRANSFORM& respectToFixed() const;
        void respectToFixed(TRANSFORM aCoordinate);
        
        TRANSFORM respectToWorld();
        void respectToWorld(TRANSFORM Tworld); // TODO: Make this move around the anchor probably
        
        JACOBIAN Jacobian(IntArray jointArray, TRANSLATION location);
        JACOBIAN Jacobian(IntArray jointArray, TRANSLATION location, Frame& refFrame);
        
        void jacobian(Eigen::MatrixXd& J, std::vector<Joint*>& jointFrames, TRANSLATION location, Frame* refFrame)__attribute__((deprecated));
        
        void updateFrames();
        void printInfo();
        
        //--------------------------------------------------------------------------
        // Kinematics Solvers
        //--------------------------------------------------------------------------

        rk_result_t selectivelyDampedLeastSquaresIK_chain(const IntArray &jointIndices, Eigen::VectorXd &jointValues,
                                         const TRANSFORM &target, const TRANSFORM &finalTF = TRANSFORM::Identity());

        rk_result_t selectivelyDampedLeastSquaresIK_chain(const std::vector<std::string>& jointNames, Eigen::VectorXd& jointValues,
                                         const TRANSFORM& target, const TRANSFORM &finalTF = TRANSFORM::Identity());

        rk_result_t selectivelyDampedLeastSquaresIK_linkage(const std::string linkageName, Eigen::VectorXd &jointValues,
                                         const TRANSFORM& target, const TRANSFORM &finalTF = TRANSFORM::Identity());


        //////////////////

        rk_result_t pseudoinverseIK_chain(const IntArray &jointIndices, Eigen::VectorXd &jointValues,
                                          const TRANSFORM &target, const TRANSFORM &finalTF = TRANSFORM::Identity());

        rk_result_t pseudoinverseIK_chain(const std::vector<std::string>& jointNames, Eigen::VectorXd& jointValues,
                                          const TRANSFORM& target, const TRANSFORM &finalTF = TRANSFORM::Identity());

        rk_result_t pseudoinverseIK_linkage(const std::string linkageName, Eigen::VectorXd &jointValues,
                                            const TRANSFORM& target, const TRANSFORM &finalTF = TRANSFORM::Identity());

        //////////////////

        rk_result_t jacobianTransposeIK_chain(const IntArray &jointIndices, Eigen::VectorXd &jointValues,
                                          const TRANSFORM &target, const TRANSFORM &finalTF = TRANSFORM::Identity());

        rk_result_t jacobianTransposeIK_chain(const std::vector<std::string>& jointNames, Eigen::VectorXd& jointValues,
                                          const TRANSFORM& target, const TRANSFORM &finalTF = TRANSFORM::Identity());

        rk_result_t jacobianTransposeIK_linkage(const std::string linkageName, Eigen::VectorXd &jointValues,
                                            const TRANSFORM& target, const TRANSFORM &finalTF = TRANSFORM::Identity());

        /////////////////

        rk_result_t dampedLeastSquaresIK_chain(const IntArray &jointIndices, Eigen::VectorXd &jointValues,
                                               const TRANSFORM &target, RobotKin::Constraints& constraints=RobotKin::Constraints::Defaults());

        rk_result_t dampedLeastSquaresIK_chain(const std::vector<std::string>& jointNames, Eigen::VectorXd& jointValues,
                                               const TRANSFORM& target, RobotKin::Constraints& constraints=RobotKin::Constraints::Defaults());

        rk_result_t dampedLeastSquaresIK_linkage(const std::string linkageName, Eigen::VectorXd &jointValues,
                                                 const TRANSFORM& target, RobotKin::Constraints& constraints=RobotKin::Constraints::Defaults());

        /////////////////

        TRANSLATION centerOfMass(FrameType withRespectTo=ROBOT); // Center of mass for entire robot + tools
        double mass();              // Mass of entire robot + tools
        TRANSLATION centerOfMass(const IntArray &indices, FrameType typeOfIndex=JOINT, FrameType withRespectTo=WORLD);
        //^ Type Of Indices can be Joint or Linkage
        TRANSLATION centerOfMass(const std::vector<std::string> &names, FrameType typeOfIndex=JOINT, FrameType withRespectTo=WORLD);
        //^ Type of Indices can be Joint or Linkage
        double mass(const IntArray &indices, FrameType typeOfIndex=JOINT);
        //^ Type of Indices can be Joint or Linkage
        double mass(const std::vector<std::string> &names, FrameType typeOfIndex=JOINT);
        //^ Type of Indices can be Joint or Linkage

        /////////////////

        void gravityJointTorques(const IntArray &jointIndices, Eigen::VectorXd &torques, bool downstream=true);


        static Robot& Default();
        
        size_t anchorJoint();
        void anchorJoint(size_t newAnchor);
        void anchorJoint(std::string newAnchor);
        size_t anchorLinkageID();

    protected:
        //--------------------------------------------------------------------------
        // Robot Protected Member Functions
        //--------------------------------------------------------------------------
        TRANSFORM respectToWorld_; // Coordinates with respect to robot base frame
        std::vector<Linkage*> linkages_;
        std::map<std::string, size_t> linkageNameToIndex_;
        std::vector<Joint*> joints_;
        std::map<std::string, size_t> jointNameToIndex_;
        
        
        //--------------------------------------------------------------------------
        // Robot Protected Member Variables
        //--------------------------------------------------------------------------
        virtual void initialize(std::vector<Linkage> linkageObjs, std::vector<int> parentIndices);
        
        
    private:
        
        size_t anchorJoint_;
        size_t anchorLinkage_;
        //--------------------------------------------------------------------------
        // Robot Private Member Variables
        //--------------------------------------------------------------------------
        bool initializing_;
        
        
        
    }; // class Robot
    
    //------------------------------------------------------------------------------
    // Postfix Increment Operators
    //------------------------------------------------------------------------------
    
} // namespace RobotKin

#endif


