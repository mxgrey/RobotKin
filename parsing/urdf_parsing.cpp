#ifdef HAVE_URDF_PARSE

#include <urdf_parser/urdf_parser.h>
#include <urdf_model/model.h>
#include <urdf_model/link.h>
#include <urdf_model/joint.h>

#include <string>
#include <fstream>
#include <boost/shared_ptr.hpp>

#include "urdf_parsing.h"

namespace RobotKinURDF{

bool exploreLink(Robot& robot, boost::shared_ptr<urdf::ModelInterface> model, boost::shared_ptr<urdf::Link> link, int id, int pID);
bool addURDFJoint(Linkage& linkage, boost::shared_ptr<urdf::Joint> ujoint);
//int findRoot(urdf::ModelInterface model);

}



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
        if( links[i]->child_joints.size() > 0 ) {
            std::cout << "\t -- with child joints: " << links[i]->child_joints[0]->name;
            for(size_t c=1; c<links[i]->child_joints.size(); c++)
                std::cout << ", " << links[i]->child_joints[c]->name;
            std::cout << std::endl;
        } else {
            std::cout << "\t -- with NO child joint. Probably this is an end link" << std::endl;
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


    boost::shared_ptr<urdf::Link> rootLink = model->root_link_;

    if(robot.name().compare("")==0)
        robot.name(model->getName());

    return exploreLink(robot, model, rootLink, 0, -1);
}




bool RobotKinURDF::exploreLink(Robot &robot, boost::shared_ptr<urdf::ModelInterface> model, boost::shared_ptr<urdf::Link> link, int id, int pID)
{
    Linkage linkage(Isometry3d::Identity(), link->name, id);

    if(link->parent_joint)
        addURDFJoint(linkage, link->parent_joint);


    if(link->child_joints.size()==1)
    {
        bool serial=true;
        boost::shared_ptr<urdf::Joint> childJoint = link->child_joints[0];
        boost::shared_ptr<urdf::Link> childLink;
        do
        {
            addURDFJoint(linkage, childJoint);

            model->getLink(childJoint->child_link_name, childLink);
            if(childLink->child_joints.size()==1)
                childJoint = childLink->child_joints[0];
            else
                serial = false;

        } while(serial);

        robot.addLinkage(linkage, pID, linkage.name());
        for(size_t i=0; i<childLink->child_joints.size(); i++)
        {
            boost::shared_ptr<urdf::Joint> nextJoint = childLink->child_joints[i];
            boost::shared_ptr<urdf::Link> nextLink;
            model->getLink(nextJoint->child_link_name, nextLink);
            size_t nextID = robot.nLinkages();
            exploreLink(robot, model, nextLink, (int)nextID, (int)id);
        }
    }
    else if(link->child_joints.size()>1)
    {
        robot.addLinkage(linkage, pID, linkage.name());
        for(size_t i=0; i<link->child_joints.size(); i++)
        {
            boost::shared_ptr<urdf::Joint> nextJoint = link->child_joints[i];
            boost::shared_ptr<urdf::Link> nextLink;
            model->getLink(nextJoint->child_link_name, nextLink);
            size_t nextID = robot.nLinkages();
            exploreLink(robot, model, nextLink, (int)nextID, (int)id);
        }
    }
    else
        robot.addLinkage(linkage, pID, linkage.name());
}

bool RobotKinURDF::addURDFJoint(Linkage &linkage, boost::shared_ptr<urdf::Joint> ujoint)
{
    Isometry3d transform(Isometry3d::Identity());
    Vector3d translation(ujoint->parent_to_joint_origin_transform.position.x,
                         ujoint->parent_to_joint_origin_transform.position.y,
                         ujoint->parent_to_joint_origin_transform.position.z);
    transform.translate(translation);

    double w, x, y, z;
    ujoint->parent_to_joint_origin_transform.rotation.getQuaternion(x,y,z,w);
    Quaterniond quat(w,x,y,z);
    transform.rotate(quat);

    RobotKin::JointType jt = REVOLUTE;
    if(ujoint->type == urdf::Joint::REVOLUTE)
        jt = REVOLUTE;
    else if(ujoint->type == urdf::Joint::PRISMATIC)
        jt = PRISMATIC;
    else
        jt = ANCHOR;

    Vector3d jointAxis(ujoint->axis.x, ujoint->axis.y, ujoint->axis.z);
    if(jointAxis.norm() == 0)
        jointAxis = Eigen::Vector3d::UnitZ();
    else
        jointAxis.normalize();

    if(ujoint->limits)
    {
        Linkage::Joint joint(transform, ujoint->name, 0, jt, jointAxis, ujoint->limits->lower, ujoint->limits->upper);
        linkage.addJoint(joint);
    }
    else
    {
        Linkage::Joint joint(transform, ujoint->name, 0, jt, jointAxis);
        linkage.addJoint(joint);
    }

}



#endif // HAVE_URDF_PARSE
