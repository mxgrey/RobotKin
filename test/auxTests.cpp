

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

void ostreamTests()
{
    rk_result_t resultTest = RK_SOLVED;
    
    cout << "RK_SOLVED:   " << resultTest << endl;
    cout << "RK_DIVERGED: " << RK_DIVERGED << endl;
    
    FrameType frameTest = JOINT;
    
    cout << "JOINT: " << frameTest << endl;
    cout << "TOOL:  " << TOOL << endl;
    
    JointType jointTest = REVOLUTE;
    
    cout << "REVOLUTE: " << jointTest << endl;
    cout << "DUMMY:    " << DUMMY << endl;
    
    StreamType streamTest = UPSTREAM;
    
    cout << "UPSTREAM: " << streamTest << endl;
    cout << "ANCHOR:   " << ANCHOR << endl;
}



int main(int argc, char *argv[])
{
    ostreamTests();

    return 0;
}
