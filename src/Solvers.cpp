
#include "Robot.h"
#include <eigen3/Eigen/SVD>

typedef Matrix<double, 6, 1> Vector6d;

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

    cout << "\n\n" << endl;
    jacobian(J, joints, joints[joints.size()-1]->respectToRobot().translation()+finalTF.translation(), this);

    JacobiSVD<MatrixXd> svd;
    svd.compute(J, ComputeFullU | ComputeThinV);

    cout <<  "\n\n" << svd.matrixU() << "\n\n\n" << svd.singularValues().transpose() << "\n\n\n" << svd.matrixV() << endl;

    for(int i=0; i<svd.matrixU().cols(); i++)
        cout << "u" << i << " : " << svd.matrixU().col(i).transpose() << endl;


    AngleAxisd aatest(target.rotation());
    cout << "AA: " << aatest.angle() << " : " << aatest.axis().transpose() << endl;

    Vector6d goal;

    goal << target.translation(), aatest.axis()*aatest.angle();

    cout << "Goal: " << goal.transpose() << endl;



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


rk_result_t Robot::dampedLeastSquaresIK_linkage(const string linkageName, vector<double> &jointValues,
                                                const Isometry3d &target, const Isometry3d &finalTF)
{
    vector<size_t> jointIndices;
    jointIndices.resize(linkage(linkageName).joints_.size());
    for(size_t i=0; i<linkage(linkageName).joints_.size(); i++)
        jointIndices[i] = linkage(linkageName).joints_[i]->id();

    Isometry3d linkageFinalTF;
    linkageFinalTF = linkage(linkageName).tool().respectToFixed()*finalTF;



    return dampedLeastSquaresIK_chain(jointIndices, jointValues, target, linkageFinalTF);
}












} // namespace RobotKin

