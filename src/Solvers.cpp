
#include "Robot.h"
#include <eigen3/Eigen/SVD>

typedef Matrix<double, 6, 1> Vector6d;

namespace RobotKin {


void clampMag(VectorXd& v, double clamp)
{
    if(v.norm() > clamp)
        v *= clamp/v.norm();
}

void clampMaxAbs(VectorXd& v, double clamp)
{
    int max=0;
    for(int i=0; i<v.size(); i++)
    {
        if(v[i] > v[max])
            max = i;
    }

    if(v[max]>clamp)
        v *= clamp/v[max];
}

double minimum(double a, double b) { return a<b ? a : b;}

// Based on a paper by Samuel R. Buss and Jin-Su Kim // TODO: Cite the paper properly
rk_result_t Robot::dampedLeastSquaresIK_chain(const vector<size_t> &jointIndices, VectorXd &jointValues,
                                              const Isometry3d &target, const Isometry3d &finalTF)
{
    // Arbitrary constant for maximum angle change in one step
    gammaMax = M_PI/4; // TODO: Put this in the constructor so the user can change it at a whim

    MatrixXd J;

    vector<Linkage::Joint*> joints;
    joints.resize(jointIndices.size());
    // FIXME: Add in safety checks
    for(int i=0; i<joints.size(); i++)
        joints[i] = joints_[jointIndices[i]];
    VectorXd initVals(joints.size());
    for(int i=0; i<joints.size(); i++)
        initVals[i] = joints[i]->value();

    cout << "\n\n" << endl;
    jacobian(J, joints, joints[joints.size()-1]->respectToRobot().translation()+finalTF.translation(), this);

    JacobiSVD<MatrixXd> svd;
    svd.compute(J, ComputeFullU | ComputeThinV);

    cout <<  "\n\n" << svd.matrixU() << "\n\n\n" << svd.singularValues().transpose() << "\n\n\n" << svd.matrixV() << endl;

    for(int i=0; i<svd.matrixU().cols(); i++)
        cout << "u" << i << " : " << svd.matrixU().col(i).transpose() << endl;



    AngleAxisd aagoal(target.rotation());
    Vector6d goal;

    goal << target.translation(), aagoal.axis()*aagoal.angle();


    Vector6d alpha;
    for(int i=0; i<6; i++)
        alpha[i] = svd.matrixU().col(i).dot(goal);

    std::cout << "Alpha: " << alpha.transpose() << std::endl;

    Vector6d N;
    for(int i=0; i<6; i++)
    {
        N[i] = svd.matrixU().block(0,i,3,1).norm();
        N[i] += svd.matrixU().block(3,i,3,1).norm();
    }

    std::cout << "N: " << N.transpose() << std::endl;

    VectorXd M(svd.matrixV().cols());
    double tempMik = 0;
    for(int i=0; i<svd.matrixV().cols(); i++)
    {
        M[i] = 0;
        for(int k=0; k<svd.matrixU().cols(); k++)
        {
            tempMik = 0;
            for(int j=0; j<svd.matrixV().cols(); j++)
                tempMik += fabs(svd.matrixV()(j,i))*J(k,j);
            M[i] += 1/svd.singularValues()[i]*tempMik;
        }
    }

    std::cout << "M: " << M.transpose() << std::endl;

    VectorXd gamma(svd.matrixV().cols());
    for(int i=0; i<svd.matrixV().cols(); i++)
        gamma[i] = minimum(1, N[i]/M[i])*gammaMax;

    std::cout << "Gamma: " << gamma.transpose() << std::endl;

    VectorXd delta(svd.matrixV().rows());
    delta.setZero();
    VectorXd tempPhi(svd.matrixV().rows());
    for(int i=0; i<svd.matrixV().cols(); i++)
    {
        std::cout << "1/sigma: " << 1/svd.singularValues()[i] << std::endl;
        tempPhi = 1/svd.singularValues()[i]*alpha[i]*svd.matrixV().col(i);
        std::cout << "Phi: " << tempPhi.transpose() << std::endl;
        clampMaxAbs(tempPhi, gamma[i]);
        delta += tempPhi;
        std::cout << "delta: " << delta.transpose() << std::endl;
    }

    clampMaxAbs(delta, gammaMax);
    std::cout << "Final delta: " << delta.transpose() << std::endl;
}

rk_result_t Robot::dampedLeastSquaresIK_chain(const vector<string> &jointNames, VectorXd &jointValues,
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


rk_result_t Robot::dampedLeastSquaresIK_linkage(const string linkageName, VectorXd &jointValues,
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

