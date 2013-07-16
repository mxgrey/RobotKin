
#include "Robot.h"
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/QR>

using namespace std;
using namespace Eigen;

typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;

namespace RobotKin {


void clampMag(VectorXd& v, double clamp)
{
    if(v.norm() > clamp)
        v *= clamp/v.norm();
}

void clampMag(Vector6d& v, double clamp)
{
    if(v.norm() > clamp)
        v *= clamp/v.norm();
}

void clampMag(Vector3d& v, double clamp)
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

double minimum(double a, double b) { return a<b ? a : b; }


// Derived from code by Yohann Solaro ( http://listengine.tuxfamily.org/lists.tuxfamily.org/eigen/2010/01/msg00187.html )
void pinv(const MatrixXd &b, MatrixXd &a_pinv)
{
    // see : http://en.wikipedia.org/wiki/Moore-Penrose_pseudoinverse#The_general_case_and_the_SVD_method

    // TODO: Figure out why it wants fewer rows than columns
//    if ( a.rows()<a.cols() )
//        return false;
    bool flip = false;
    MatrixXd a;
    if( a.rows() < a.cols() )
    {
        a = b.transpose();
        flip = true;
    }
    else
        a = b;


    // SVD
    JacobiSVD< MatrixXd > svdA;
    svdA.compute(a, ComputeFullU | ComputeThinV);

    JacobiSVD<MatrixXd>::SingularValuesType vSingular = svdA.singularValues();



    // Build a diagonal matrix with the Inverted Singular values
    // The pseudo inverted singular matrix is easy to compute :
    // is formed by replacing every nonzero entry by its reciprocal (inversing).
    VectorXd vPseudoInvertedSingular(svdA.matrixV().cols());


    for (int iRow =0; iRow<vSingular.rows(); iRow++)
    {
        if ( fabs(vSingular(iRow))<=1e-10 ) // Todo : Put epsilon in parameter
        {
            vPseudoInvertedSingular(iRow)=0.;
        }
        else
        {
            vPseudoInvertedSingular(iRow)=1./vSingular(iRow);
        }

    }



    // A little optimization here
    MatrixXd mAdjointU = svdA.matrixU().adjoint().block(0,0,vSingular.rows(),svdA.matrixU().adjoint().cols());


    // Pseudo-Inversion : V * S * U'
    a_pinv = (svdA.matrixV() *  vPseudoInvertedSingular.asDiagonal()) * mAdjointU;


    if(flip)
    {
        a = a.transpose();
        a_pinv = a_pinv.transpose();
    }
}



// Based on a paper by Samuel R. Buss and Jin-Su Kim // TODO: Cite the paper properly
rk_result_t Robot::selectivelyDampedLeastSquaresIK_chain(const vector<size_t> &jointIndices, VectorXd &jointValues,
                                              const Isometry3d &target, const Isometry3d &finalTF)
{
    return RK_SOLVER_NOT_READY;
    // FIXME: Make this work


    // Arbitrary constant for maximum angle change in one step
    gammaMax = M_PI/4; // TODO: Put this in the constructor so the user can change it at a whim


    vector<Linkage::Joint*> joints;
    joints.resize(jointIndices.size());
    // FIXME: Add in safety checks
    for(int i=0; i<joints.size(); i++)
        joints[i] = joints_[jointIndices[i]];

    // ~~ Declarations ~~
    MatrixXd J;
    JacobiSVD<MatrixXd> svd;
    Isometry3d pose;
    AngleAxisd aagoal;
    AngleAxisd aastate;
    Vector6d goal;
    Vector6d state;
    Vector6d err;
    Vector6d alpha;
    Vector6d N;
    Vector6d M;
    Vector6d gamma;
    VectorXd delta(jointValues.size());
    VectorXd tempPhi(jointValues.size());
    // ~~~~~~~~~~~~~~~~~~

//    cout << "\n\n" << endl;

    tolerance = 1*M_PI/180; // TODO: Put this in the constructor so the user can set it arbitrarily
    maxIterations = 1000; // TODO: Put this in the constructor so the user can set it arbitrarily

    size_t iterations = 0;
    do {

        values(jointIndices, jointValues);

        jacobian(J, joints, joints.back()->respectToRobot().translation()+finalTF.translation(), this);

        svd.compute(J, ComputeFullU | ComputeThinV);

    //    cout <<  "\n\n" << svd.matrixU() << "\n\n\n" << svd.singularValues().transpose() << "\n\n\n" << svd.matrixV() << endl;

    //    for(int i=0; i<svd.matrixU().cols(); i++)
    //        cout << "u" << i << " : " << svd.matrixU().col(i).transpose() << endl;


    //    std::cout << "Joint name: " << joint(jointIndices.back()).name()
    //              << "\t Number: " << jointIndices.back() << std::endl;
        pose = joint(jointIndices.back()).respectToRobot()*finalTF;

    //    std::cout << "Pose: " << std::endl;
    //    std::cout << pose.matrix() << std::endl;

    //    AngleAxisd aagoal(target.rotation());
        aagoal = target.rotation();
        goal << target.translation(), aagoal.axis()*aagoal.angle();

        aastate = pose.rotation();
        state << pose.translation(), aastate.axis()*aastate.angle();

        err = goal-state;

    //    std::cout << "state: " << state.transpose() << std::endl;
    //    std::cout << "err: " << err.transpose() << std::endl;

        for(int i=0; i<6; i++)
            alpha[i] = svd.matrixU().col(i).dot(err);

    //    std::cout << "Alpha: " << alpha.transpose() << std::endl;

        for(int i=0; i<6; i++)
        {
            N[i] = svd.matrixU().block(0,i,3,1).norm();
            N[i] += svd.matrixU().block(3,i,3,1).norm();
        }

    //    std::cout << "N: " << N.transpose() << std::endl;

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

    //    std::cout << "M: " << M.transpose() << std::endl;

        for(int i=0; i<svd.matrixV().cols(); i++)
            gamma[i] = minimum(1, N[i]/M[i])*gammaMax;

    //    std::cout << "Gamma: " << gamma.transpose() << std::endl;

        delta.setZero();
        for(int i=0; i<svd.matrixV().cols(); i++)
        {
    //        std::cout << "1/sigma: " << 1/svd.singularValues()[i] << std::endl;
            tempPhi = 1/svd.singularValues()[i]*alpha[i]*svd.matrixV().col(i);
    //        std::cout << "Phi: " << tempPhi.transpose() << std::endl;
            clampMaxAbs(tempPhi, gamma[i]);
            delta += tempPhi;
    //        std::cout << "delta " << i << ": " << delta.transpose() << std::endl;
        }

        clampMaxAbs(delta, gammaMax);

        jointValues += delta;

        std::cout << iterations << " | Norm:" << delta.norm() << "\tdelta: "
                  << delta.transpose() << "\tJoints:" << jointValues.transpose() << std::endl;

        iterations++;
    } while(delta.norm() > tolerance && iterations < maxIterations);
}

rk_result_t Robot::selectivelyDampedLeastSquaresIK_chain(const vector<string> &jointNames, VectorXd &jointValues,
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

    return selectivelyDampedLeastSquaresIK_chain(jointIndices, jointValues, target);
}


rk_result_t Robot::selectivelyDampedLeastSquaresIK_linkage(const string linkageName, VectorXd &jointValues,
                                                const Isometry3d &target, const Isometry3d &finalTF)
{
    vector<size_t> jointIndices;
    jointIndices.resize(linkage(linkageName).joints_.size());
    for(size_t i=0; i<linkage(linkageName).joints_.size(); i++)
        jointIndices[i] = linkage(linkageName).joints_[i]->id();

    Isometry3d linkageFinalTF;
    linkageFinalTF = linkage(linkageName).tool().respectToFixed()*finalTF;

    return selectivelyDampedLeastSquaresIK_chain(jointIndices, jointValues, target, linkageFinalTF);
}



rk_result_t Robot::pseudoinverseIK_chain(const vector<size_t> &jointIndices, VectorXd &jointValues,
                                  const Isometry3d &target, const Isometry3d &finalTF)
{
    return RK_SOLVER_NOT_READY;
    // FIXME: Make this solver work


    vector<Linkage::Joint*> joints;
    joints.resize(jointIndices.size());
    // FIXME: Add in safety checks
    for(int i=0; i<joints.size(); i++)
        joints[i] = joints_[jointIndices[i]];

    // ~~ Declarations ~~
    MatrixXd J;
    MatrixXd Jinv;
    Isometry3d pose;
    AngleAxisd aagoal;
    AngleAxisd aastate;
    Vector6d goal;
    Vector6d state;
    Vector6d err;
    VectorXd delta(jointValues.size());

    MatrixXd Jsub;
    aagoal = target.rotation();
    goal << target.translation(), aagoal.axis()*aagoal.angle();

    tolerance = 1*M_PI/180; // TODO: Put this in the constructor so the user can set it arbitrarily
    maxIterations = 100; // TODO: Put this in the constructor so the user can set it arbitrarily
    errorClamp = 0.25; // TODO: Put this in the constructor
    deltaClamp = M_PI/4; // TODO: Put this in the constructor

    size_t iterations = 0;
    do {

        values(jointIndices, jointValues);

        jacobian(J, joints, joints.back()->respectToRobot().translation()+finalTF.translation(), this);
        Jsub = J.block(0,0,3,jointValues.size());

        pinv(Jsub, Jinv);

        pose = joint(jointIndices.back()).respectToRobot()*finalTF;
        aastate = pose.rotation();
        state << pose.translation(), aastate.axis()*aastate.angle();

        err = goal-state;
        for(int i=3; i<6; i++)
            err[i] *= 0;
        err.normalize();

        Vector3d e = (target.translation() - pose.translation()).normalized()*0.005;

//        delta = Jinv*err*0.1;
//        clampMag(delta, deltaClamp);
        VectorXd d = Jinv*e;

//        jointValues += delta;
        jointValues += d;
        std::cout << iterations << " | Norm:" << delta.norm()
//                  << "\tdelta: " << delta.transpose() << "\tJoints:" << jointValues.transpose() << std::endl;
                  << " | " << (target.translation() - pose.translation()).norm()
                  << "\tErr: " << (goal-state).transpose() << std::endl;


        iterations++;
    } while(delta.norm() > tolerance && iterations < maxIterations);

}



rk_result_t Robot::pseudoinverseIK_chain(const vector<string> &jointNames, VectorXd &jointValues,
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

    return pseudoinverseIK_chain(jointIndices, jointValues, target);
}


rk_result_t Robot::pseudoinverseIK_linkage(const string linkageName, VectorXd &jointValues,
                                                const Isometry3d &target, const Isometry3d &finalTF)
{
    vector<size_t> jointIndices;
    jointIndices.resize(linkage(linkageName).joints_.size());
    for(size_t i=0; i<linkage(linkageName).joints_.size(); i++)
        jointIndices[i] = linkage(linkageName).joints_[i]->id();

    Isometry3d linkageFinalTF;
    linkageFinalTF = linkage(linkageName).tool().respectToFixed()*finalTF;

    return pseudoinverseIK_chain(jointIndices, jointValues, target, linkageFinalTF);
}


rk_result_t Robot::jacobianTransposeIK_chain(const vector<size_t> &jointIndices, VectorXd &jointValues, const Isometry3d &target, const Isometry3d &finalTF)
{
    return RK_SOLVER_NOT_READY;
    // FIXME: Make this solver work


    vector<Linkage::Joint*> joints;
    joints.resize(jointIndices.size());
    // FIXME: Add in safety checks
    for(int i=0; i<joints.size(); i++)
        joints[i] = joints_[jointIndices[i]];

    // ~~ Declarations ~~
    MatrixXd J;
    MatrixXd Jinv;
    Isometry3d pose;
    AngleAxisd aagoal;
    AngleAxisd aastate;
    Vector6d state;
    Vector6d err;
    VectorXd delta(jointValues.size());
    Vector6d gamma;
    double alpha;

    aagoal = target.rotation();

    double Tscale = 3; // TODO: Put these as a class member in the constructor
    double Rscale = 0;

    tolerance = 1*M_PI/180; // TODO: Put this in the constructor so the user can set it arbitrarily
    maxIterations = 100; // TODO: Put this in the constructor so the user can set it arbitrarily

    size_t iterations = 0;
    do {
        values(jointIndices, jointValues);

        jacobian(J, joints, joints.back()->respectToRobot().translation()+finalTF.translation(), this);

        pose = joint(jointIndices.back()).respectToRobot()*finalTF;
        aastate = pose.rotation();
        state << pose.translation(), aastate.axis()*aastate.angle();

        err << (target.translation()-pose.translation()).normalized()*Tscale,
               (aagoal.angle()*aagoal.axis()-aastate.angle()*aastate.axis()).normalized()*Rscale;

        gamma = J*J.transpose()*err;
        alpha = err.dot(gamma)/gamma.norm();

        delta = alpha*J.transpose()*err;

        jointValues += delta;
        iterations++;

        std::cout << iterations << " | Norm:" << delta.norm()
//                  << "\tdelta: " << delta.transpose() << "\tJoints:" << jointValues.transpose() << std::endl;
                  << " | " << (target.translation() - pose.translation()).norm()
                  << "\tErr: " << (target.translation()-pose.translation()).transpose() << std::endl;

    } while(err.norm() > tolerance && iterations < maxIterations);

}


rk_result_t Robot::jacobianTransposeIK_chain(const vector<string> &jointNames, VectorXd &jointValues,
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

    return jacobianTransposeIK_chain(jointIndices, jointValues, target);
}


rk_result_t Robot::jacobianTransposeIK_linkage(const string linkageName, VectorXd &jointValues,
                                                const Isometry3d &target, const Isometry3d &finalTF)
{
    vector<size_t> jointIndices;
    jointIndices.resize(linkage(linkageName).joints_.size());
    for(size_t i=0; i<linkage(linkageName).joints_.size(); i++)
        jointIndices[i] = linkage(linkageName).joints_[i]->id();

    Isometry3d linkageFinalTF;
    linkageFinalTF = linkage(linkageName).tool().respectToFixed()*finalTF;

    return jacobianTransposeIK_chain(jointIndices, jointValues, target, linkageFinalTF);
}




rk_result_t Robot::dampedLeastSquaresIK_chain(const vector<size_t> &jointIndices, VectorXd &jointValues, const Isometry3d &target, const Isometry3d &finalTF)
{


    vector<Linkage::Joint*> joints;
    joints.resize(jointIndices.size());
    // FIXME: Add in safety checks
    for(int i=0; i<joints.size(); i++)
        joints[i] = joints_[jointIndices[i]];

    // ~~ Declarations ~~
    MatrixXd J;
    MatrixXd Jinv;
    Isometry3d pose;
    AngleAxisd aagoal(target.rotation());
    AngleAxisd aastate;
    Vector3d Terr;
    Vector3d Rerr;
    Vector6d err;
    VectorXd delta(jointValues.size());
    VectorXd f(jointValues.size());


    tolerance = 0.001;
    maxIterations = 50; // TODO: Put this in the constructor so the user can set it arbitrarily
    damp = 0.05;

    values(jointIndices, jointValues);

    pose = joint(jointIndices.back()).respectToRobot()*finalTF;
    aastate = pose.rotation();

    Terr = target.translation()-pose.translation();
    Rerr = aagoal.angle()*aagoal.axis()-aastate.angle()*aastate.axis();
    err << Terr, Rerr;

    size_t iterations = 0;
    do {

        jacobian(J, joints, joints.back()->respectToRobot().translation()+finalTF.translation(), this);

        f = (J*J.transpose() + damp*damp*Matrix6d::Identity()).colPivHouseholderQr().solve(err);
        delta = J.transpose()*f;

        jointValues += delta;

        values(jointIndices, jointValues);

        pose = joint(jointIndices.back()).respectToRobot()*finalTF;
        aastate = pose.rotation();

        Terr = target.translation()-pose.translation();
        Rerr = aagoal.angle()*aagoal.axis()-aastate.angle()*aastate.axis();
        err << Terr, Rerr;

        iterations++;


    } while(err.norm() > tolerance && iterations < maxIterations);

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

