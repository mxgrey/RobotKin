#ifndef URDF_PARSING_H
#define URDF_PARSING_H

#include "Robot.h"


namespace RobotKinURDF{

bool loadURDF(RobotKin::Robot& robot, std::string filename);
bool loadURDFstring(RobotKin::Robot& robot, std::string xml_model_string);

}




#endif // URDF_PARSING_H
