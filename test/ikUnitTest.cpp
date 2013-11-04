


//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include <iostream>
#include <vector>
#include "Frame.h"
#include "Linkage.h"
#include "Robot.h"
#include "Hubo.h"

#include <time.h>



//------------------------------------------------------------------------------
// Namespaces
//------------------------------------------------------------------------------
using namespace std;
using namespace Eigen;
using namespace RobotKin;












void ikTest(bool totallyRandom)
{
    cout << "-------------------------------------------" << endl;
    cout << "| Testing Linkage Damped Least Squares IK |" << endl;
    cout << "-------------------------------------------" << endl;


    Robot ikTest("../urdf/huboplus.urdf");
    ikTest.linkage("Body_RSP").name("RightArm");
    ikTest.linkage("Body_LSP").name("LeftArm");
    ikTest.linkage("Body_RHY").name("RightLeg");
    ikTest.linkage("Body_LHY").name("LeftLeg");

//    string limb = "LeftArm";
    string limb = "RightArm";

    VectorXd targetValues, jointValues, restValues, storedVals;
    targetValues.resize(ikTest.linkage(limb).nJoints());
    jointValues.resize(ikTest.linkage(limb).nJoints());
    restValues.resize(ikTest.linkage(limb).nJoints());
    restValues.setZero();

    if(limb == "RightLeg" || limb == "LeftLeg")
    {
        restValues[0] = 0;
        restValues[1] = 0;
        restValues[2] = -0.15;
        restValues[3] = 0.3;
        restValues[4] = -0.15;
        restValues[5] = 0;
    }
    else if(limb == "RightArm" || limb == "LeftArm")
    {
        restValues[0] = 0;
        if(limb=="RightArm")
            restValues[1] = -30*M_PI/180;
        else
            restValues[1] =  30*M_PI/180;
        restValues[2] = 0;
        restValues[3] = -30*M_PI/180;
        restValues[4] = 0;
        restValues[5] = 0;
    }

//    bool totallyRandom = true;
    int resolution = 1000;
    double scatterScale = 0.05;
    
    int tests = 5000;


    Constraints constraints;
    constraints.restingValues(restValues);
    if(!totallyRandom)
    {
        constraints.performNullSpaceTask = false;
        constraints.maxAttempts = 1;
        constraints.maxIterations = 50;
//        constraints.convergenceTolerance = 0.001;
//    constraints.wrapToJointLimits = true;
//    constraints.wrapSolutionToJointLimits = true;
        constraints.wrapToJointLimits = false;
        constraints.wrapSolutionToJointLimits = false;
    }
    else
    {
//        constraints.performNullSpaceTask = true;
//        constraints.maxAttempts = 3;
//        constraints.maxIterations = 200;
//        constraints.convergenceTolerance = 0.001;
//        constraints.wrapToJointLimits = true;
//        constraints.wrapSolutionToJointLimits = true;
//        constraints.wrapToJointLimits = false;
//        constraints.wrapSolutionToJointLimits = false;
    }

    TRANSFORM target;

    int wins = 0, jumps = 0;

    srand(time(NULL));
    
    clock_t time;
    time = clock();
    

    for(int k=0; k<tests; k++)
    {
        for(int i=0; i<ikTest.linkage(limb).nJoints(); i++)
        {
            int randomVal = rand();
            targetValues(i) = ((double)(randomVal%resolution))/((double)resolution-1)
                    *(ikTest.linkage(limb).joint(i).max() - ikTest.linkage(limb).joint(i).min())
                    + ikTest.linkage(limb).joint(i).min();

    //        cout << "vals: " << randomVal;

            randomVal =  rand();

//            cout << "\t:\t" << randomVal << endl;

            /////////////////////// TOTALLY RANDOM TEST ///////////////////////////
            if(totallyRandom)
                jointValues(i) = ((double)(randomVal%resolution))/((double)resolution-1)
                        *(ikTest.linkage(limb).joint(i).max() - ikTest.linkage(limb).joint(i).min())
                        + ikTest.linkage(limb).joint(i).min();
            ////////////////////// NOT COMPLETELY RANDOM TEST //////////////////////
            else
//                jointValues(i) = targetValues(i) +
//                        ((double)(2*rand()%resolution)/((double)resolution-1)-1)*scatterScale
//                        *(parseTest.linkage(limb).joint(i).max() - parseTest.linkage(limb).joint(i).min());
                jointValues(i) = targetValues(i) +
                        ((double)(2*rand()%resolution)/((double)resolution-1)-1)*scatterScale;

            ikTest.linkage(limb).joint(i).value(targetValues(i));
        }


        target = ikTest.linkage(limb).tool().respectToRobot();


        ikTest.linkage(limb).values(jointValues);

//        cout << "Start:" << endl;
//        cout << parseTest.linkage(limb).tool().respectToRobot().matrix() << endl;
//        cout << "Target:" << endl;
//        cout << target.matrix() << endl;
        storedVals = jointValues;

        rk_result_t result = ikTest.dampedLeastSquaresIK_linkage(limb, jointValues, target, constraints);
        if( result == RK_SOLVED )
            wins++;
        
        
//        if(false)
        if(!totallyRandom && (result != RK_SOLVED || (storedVals-jointValues).norm() > 1.1*(storedVals-targetValues).norm()) )
        {
            if( result == RK_SOLVED )
                cout << "SOLVED" << endl;
            else
            {
                Vector3d Terr = target.translation() - ikTest.linkage(limb).tool().respectToRobot().translation();
                AngleAxisd aaerr;
                aaerr = target.rotation()*ikTest.linkage(limb).tool().respectToRobot().rotation().transpose();
                Vector3d Rerr;
                if(fabs(aaerr.angle()) <= M_PI)
                    Rerr = aaerr.angle()*aaerr.axis();
                else
                    Rerr = (aaerr.angle()-2*M_PI)*aaerr.axis();
                
                cout << "FAILED (" << Terr.norm() << ", " << Rerr.norm() << ")" << endl;
            }
            cout << "Start: " << storedVals.transpose() << endl;
            cout << "Solve: " << jointValues.transpose() << endl;
            cout << "Goal : " << targetValues.transpose() << endl;
            cout << "Delta: " << (storedVals-targetValues).transpose() << "\t(" << (storedVals-targetValues).norm() << ")" << endl;
            cout << "Diff : " << (storedVals-jointValues).transpose() << "\t(" << (storedVals-jointValues).norm() << ")" << endl << endl;
            if( result == RK_SOLVED )
                jumps++;
        }
            

//        cout << "End:" << endl;
//        cout << parseTest.linkage(limb).tool().respectToRobot().matrix() << endl;

//        cout << "Target Joint Values: " << targetValues.transpose() << endl;


//        if(result != RK_SOLVED)
        if(false)
        {

            cout << "Start values:       ";
            for(int p=0; p<storedVals.size(); p++)
                cout << storedVals[p] << "\t\t";
            cout << endl;

            cout << "Target values:      ";
            for(int p=0; p<targetValues.size(); p++)
                cout << targetValues[p] << "\t\t";
            cout << endl;

            cout << "Final Joint Values: ";
            for(int p=0; p<jointValues.size(); p++)
                cout << jointValues[p] << "\t\t";
            cout << endl;


            cout << "Norm: " << ( jointValues - targetValues ).norm() << endl;
            cout << "Error: " << (ikTest.linkage(limb).tool().respectToRobot().translation()
                                  - target.translation()).norm() << endl;
        }
    }
    
    
    clock_t endTime;
    endTime = clock();
    cout << "Time: " << (endTime - time)/((double)CLOCKS_PER_SEC*tests) << " : " <<
            (double)CLOCKS_PER_SEC*tests/(endTime-time) << endl;
    

    cout << "Win: " << ((double)wins)/((double)tests)*100 << "%" << endl;

    if(!totallyRandom)
        cout << "Jump: " << ((double)jumps)/((double)tests)*100 << "%" << endl;



}







int main(int argc, char *argv[])
{
    cout << "Totally Random: " << endl;
    ikTest(true);
    
    cout << endl << endl << "Continuous Mode: " << endl;
    ikTest(false);

    return 0;
}

