#ifndef URDF_PARSING_H
#define URDF_PARSING_H

#include "Robot.h"

using namespace RobotKin;

namespace RobotKinURDF{

bool loadURDF(Robot& robot, string filename);

}




#endif // URDF_PARSING_H
