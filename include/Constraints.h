
#ifndef CONSTRAINTS_H
#define CONSTRAINTS_H

#include "Frame.h"
#include <vector>

namespace RobotKin {


    class Constraints
    {
    public:
        Constraints();

        bool performNullSpaceTask;
        virtual Eigen::VectorXd nullSpaceTask(Robot& robot, const std::vector<size_t>& indices,
                                              const Eigen::VectorXd& values, Eigen::VectorXd& nullTask);
        void restingValues(Eigen::VectorXd newRestingValues);
        Eigen::VectorXd& restingValues();

        bool performErrorClamp;
        double translationClamp;
        double rotationClamp;

        bool customErrorClamp;
        virtual void errorClamp(Robot& robot, const std::vector<size_t>& indices, SCREW& error);

        int maxIterations;
        double dampingConstant;
        double convergenceTolerance;

        TRANSFORM finalTransform;

        bool useIterativeJacobianSeed;
        virtual void iterativeJacobianSeed(Robot &robot, size_t attemptNumber,
                                           const std::vector<size_t>& indices, Eigen::VectorXd& values);
        size_t maxAttempts;


        // Allow the user to call some default constraints
        static Constraints& Defaults();

    protected:

        Eigen::VectorXd restingValues_;



    private:



    };

}























#endif // CONSTRAINTS_H
