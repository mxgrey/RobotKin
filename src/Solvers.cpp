
#include "Robot.h"


namespace RobotKin {

rk_result_t Robot::dampedLeastSquaresIK_chain(const vector<size_t> &jointIndices, vector<double> &jointValues,
                                              const Isometry3d &target, const Isometry3d &finalTF)
{
    MatrixXd J;

    vector<Linkage::Joint*> joints;
    joints.resize(jointIndices.size());
    // FIXME: Add in safety checks
    for(int i=0; i<joints.size(); i++)
        joints[i] = joints_[jointIndices[i]];
    vector<double> initVals(joints.size());
    for(int i=0; i<joints.size(); i++)
        initVals[i] = joints[i]->value();

    jacobian(J, joints, joints[joints.size()-1]->respectToRobot().translation(), this);


}

rk_result_t Robot::dampedLeastSquaresIK_chain(const vector<string> &jointNames, vector<double> &jointValues,
                                              const Isometry3d &target, const Isometry3d &finalTF)
{
    // TODO: Make the conversion from vector<string> to vector<size_t> its own function
    vector<size_t> jointIndices;
    jointIndices.resize(jointNames.size());
    map<string,size_t>::iterator j;
    for(int i=0; i<jointNames.size(); i++)
    {
        j = jointNameToIndex_.find(jointNames[i]);
        if( j == jointNameToIndex_.end() )
            return RK_INVALID_JOINT;
        jointIndices[i] = j->second;
    }

    return dampedLeastSquaresIK_chain(jointIndices, jointValues, target);
}
















} // namespace RobotKin

