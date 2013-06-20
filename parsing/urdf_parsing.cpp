
#define HAVE_URDF_PARSE // TODO: REMOVE THIS BEFORE RELEASE


#ifdef HAVE_URDF_PARSE

#include <urdf_parser/urdf_parser.h>
#include <urdf_model/model.h>
#include <urdf_model/link.h>
#include <urdf_model/joint.h>

#include <string>
#include <fstream>
#include <boost/shared_ptr.hpp>

#include "urdf_parsing.h"

using namespace RobotKinURDF;
using namespace std;

bool RobotKinURDF::loadURDF(Robot& robot, string filename)
{
  boost::shared_ptr<urdf::ModelInterface> model;

  // Read the urdf into a string
  std::string xml_model_string;
  std::fstream xml_file( filename.c_str(),
             std::fstream::in );

  while( xml_file.good() ) {
    std::string line;
    std::getline( xml_file, line );
      xml_model_string += ( line + "\n" );
  }

  xml_file.close();

  // Parse model using the urdf_parser
  model = urdf::parseURDF( xml_model_string );

  // Output some info from the urdf

  // Robot name
  std::cout << " URDF Robot name: "<< model->getName() << std::endl;

  // Links
  std::vector< boost::shared_ptr<urdf::Link> > links;
  model->getLinks( links );


  std::cout << "The robot has "<< links.size() << " links:" << std::endl;

  for( int i = 0; i < links.size(); ++i ) {
    std::cout << " Link [" << i << "]: "<< links[i]->name << std::endl;
    if( links[i]->getParent() ) {
      std::cout << "\t -- with parent joint: "<< links[i]->parent_joint->name << std::endl;
    } else {
      std::cout << "\t -- with NO parent joint. Probably this is the root link" << std::endl;
    }

    if( links[i]->inertial ) {
      std::cout << "\t -- with mass: "<< links[i]->inertial->mass << std::endl;
      std::cout << "\t -- and inertia moments:"<< links[i]->inertial->ixx<<
    ", "<<links[i]->inertial->ixy <<
    ", "<<links[i]->inertial->ixz <<
    ", "<<links[i]->inertial->iyy <<
    ", "<<links[i]->inertial->iyz <<
    ", "<<links[i]->inertial->izz <<std::endl;
    }
  }

  return true;
}






#endif // HAVE_URDF_PARSE
