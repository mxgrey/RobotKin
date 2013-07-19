

#include "Robot.h"
#include "Constraints.h"

#include <time.h>


using namespace RobotKin;
using namespace Eigen;
using namespace std;

Constraints::Constraints()
    : performNullSpaceTask(false),
      maxIterations(250),
      dampingConstant(0.05),
      finalTransform(TRANSFORM::Identity()),
      convergenceTolerance(0.001),
      performErrorClamp(true),
      translationClamp(20*convergenceTolerance),
      rotationClamp(translationClamp/10),
      customErrorClamp(false),
      useIterativeJacobianSeed(true),
      maxAttempts(5)
{

}




Constraints &Constraints::Defaults()
{
    Constraints* constraints = new Constraints;
    return *constraints;
}

void Constraints::restingValues(VectorXd newRestingValues)
{
    performNullSpaceTask = true;
    restingValues_ = newRestingValues;
}

VectorXd& Constraints::restingValues() { return restingValues_; }

VectorXd Constraints::nullSpaceTask(Robot& robot, const std::vector<size_t> &indices,
                                    const VectorXd& values, VectorXd& nullTask)
{
    nullTask = restingValues_ - values;
    clampMag(nullTask, convergenceTolerance/10.0);
//    clampMag(nullErr, 0.01);

    return nullTask;
}

void Constraints::errorClamp(Robot &robot, const std::vector<size_t> &indices, SCREW &error)
{

}

void Constraints::iterativeJacobianSeed(Robot& robot, size_t attemptNumber,
                                        const std::vector<size_t> &indices, Eigen::VectorXd &values)
{
    if( attemptNumber == 0 )
    {
        return;
    }
    else if( attemptNumber == 1 && values.size() == restingValues_.size() )
        for(int i=0; i<values.size(); i++)
            values(i) = restingValues_(i);
    else
    {
        int resolution = 1000;
        int randVal = rand();
        for(int i=0; i<values.size(); i++)
            values(i) = ((double)(randVal%resolution))/((double)resolution-1)
                    *(robot.joint(indices[i]).max() - robot.joint(indices[i]).min())
                    + robot.joint(indices[i]).min();
//        cout << randVal << endl;
    }
}






