


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


void ikTest();







int main(int argc, char *argv[])
{
    ikTest();

    return 0;
}






void ikTest()
{
    cout << "--------------------------------------------" << endl;
    cout << "| Testing Standard Damped Least Squares IK |" << endl;
    cout << "--------------------------------------------" << endl;


    Robot parseTest("../urdf/drchubo.urdf");
    parseTest.linkage("Body_RSP").name("RightArm");
    parseTest.linkage("Body_LSP").name("LeftArm");
    parseTest.linkage("Body_RHY").name("RightLeg");
    parseTest.linkage("Body_LHY").name("LeftLeg");


    bool totallyRandom = true;

    string limb = "RightLeg";

    VectorXd targetValues, jointValues, restValues, storedVals;
    targetValues.resize(parseTest.linkage(limb).nJoints());
    jointValues.resize(parseTest.linkage(limb).nJoints());
    restValues.resize(parseTest.linkage(limb).nJoints());
    restValues.setZero();
    // Leg rest values
    restValues[0] = 0;
    restValues[1] = 0;
    restValues[2] = -0.15;
    restValues[3] = 0.3;
    restValues[4] = -0.15;
    restValues[5] = 0;
    restValues[6] = 0;

    // Arm rest values

//    restValues[3] = -30*M_PI/180;


    Constraints constraints;
    constraints.restingValues(restValues);
    constraints.performNullSpaceTask = true;

    TRANSFORM target;

    int wins = 0;

    srand(time(NULL));

    int tests = 1000;
    for(int k=0; k<tests; k++)
    {
        int resolution = 1000;
        double scatterScale = 0.1;
        for(int i=0; i<parseTest.linkage(limb).nJoints(); i++)
        {
            int randomVal = rand();
            targetValues(i) = ((double)(randomVal%resolution))/((double)resolution-1)
                    *(parseTest.linkage(limb).joint(i).max() - parseTest.linkage(limb).joint(i).min())
                    + parseTest.linkage(limb).joint(i).min();

    //        cout << "vals: " << randomVal;

            randomVal =  rand();

//            cout << "\t:\t" << randomVal << endl;

            /////////////////////// TOTALLY RANDOM TEST ///////////////////////////
            if(totallyRandom)
                jointValues(i) = ((double)(randomVal%resolution))/((double)resolution-1)
                        *(parseTest.linkage(limb).joint(i).max() - parseTest.linkage(limb).joint(i).min())
                        + parseTest.linkage(limb).joint(i).min();
            ////////////////////// NOT COMPLETELY RANDOM TEST //////////////////////
            else
//                jointValues(i) = targetValues(i) +
//                        ((double)(2*rand()%resolution)/((double)resolution-1)-1)*scatterScale
//                        *(parseTest.linkage(limb).joint(i).max() - parseTest.linkage(limb).joint(i).min());
                jointValues(i) = targetValues(i) +
                        ((double)(2*rand()%resolution)/((double)resolution-1)-1)*scatterScale;

            parseTest.linkage(limb).joint(i).value(targetValues(i));
        }


        target = parseTest.linkage(limb).tool().respectToRobot();


        parseTest.linkage(limb).values(jointValues);

//        cout << "Start:" << endl;
//        cout << parseTest.linkage(limb).tool().respectToRobot().matrix() << endl;
//        cout << "Target:" << endl;
//        cout << target.matrix() << endl;
        storedVals = jointValues;

        rk_result_t result = parseTest.dampedLeastSquaresIK_linkage(limb, jointValues, target, constraints);
        if( result == RK_SOLVED )
            wins++;

//        cout << "End:" << endl;
//        cout << parseTest.linkage(limb).tool().respectToRobot().matrix() << endl;

//        cout << "Target Joint Values: " << targetValues.transpose() << endl;


        if(result != RK_SOLVED)
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
            cout << "Error: " << (parseTest.linkage(limb).tool().respectToRobot().translation()
                                  - target.translation()).norm() << endl;
        }
    }

    cout << "Win: " << ((double)wins)/((double)tests)*100 << "%" << endl;





}
