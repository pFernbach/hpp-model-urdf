// Copyright (C) 2012 by Antonio El Khoury.
//
// This file is part of the hpp-model-urdf.
//
// hpp-model-urdf is free software: you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation, either version 3 of
// the License, or (at your option) any later version.
//
// hpp-model-urdf is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with hpp-model-urdf.  If not, see
// <http://www.gnu.org/licenses/>.

#define BOOST_TEST_MODULE display-robot

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include <KineoModel/kppLicense.h>
#include <KineoModel/kppJointComponent.h>

#include "hpp/model/urdf/parser.hh"
#include "hpp/model/srdf/parser.hh"

using boost::test_tools::output_test_stream;

BOOST_AUTO_TEST_CASE (display_robot)
{
  // Validate Kineo license.
  if (!CkppLicense::initialize ())
    {
      std::cout << "Failed to validate Kineo license." << std::endl;
      return;
    }
  
  using namespace hpp::model::urdf;
  hpp::model::urdf::Parser urdfParser; 
  hpp::model::srdf::Parser srdfParser; 
  hpp::model::HumanoidRobotShPtr humanoidRobot;

  // if (argc < 2)
  //   {
  //     std::cout
  // 	<< "No model description has been given, "
  // 	<< "retrieving model using ROS parameter (robot_description)."
  // 	<< std::endl;

  //     ros::init (argc, argv, "display_robot");
  //     ros::NodeHandle nh;
  //     std::string robotDescription;
  //     ros::param::param<std::string>
  // 	("robot_description", robotDescription, "");
  //     if (robotDescription.empty ())
  // 	{
  // 	  std::cout
  // 	    << "No model available as ROS parameter. Fail."
  // 	    << std::endl;
  // 	  usage (argc, argv);
  // 	  return 1;
  // 	}
  //     humanoidRobot = parser.parseStream (robotDescription,
  // 					  "base_footprint_joint");
  //   }
  // else
  humanoidRobot = urdfParser.parse ("package://hrp2_14_description/urdf/hrp2-capsule.urdf", "base_footprint_joint");

  srdfParser.parse ("package://hrp2_14_description/urdf/hrp2-capsule.urdf",
		    "package://hrp2_14_description/srdf/hrp2.srdf",
		    humanoidRobot);

  BOOST_CHECK_EQUAL (!!humanoidRobot, 1);

  for (unsigned i = 0; i < humanoidRobot->countJointComponents (); ++i)
    {
      CkwsJointShPtr joint = humanoidRobot->jointComponent (i)->kwsJoint ();
      CkwsBodyShPtr body = joint->attachedBody ();

      if (body)
	{
	  Parser::BodyPtrType hppBody
	    = KIT_DYNAMIC_PTR_CAST (hpp::model::Body, body);
	  if (!hppBody)
	    std::cerr << "Null pointer to body." << std::endl;

	  std::cout << hppBody->nbDistPairs () << std::endl;
	}
    }

  // parser.displayActuatedJoints (std::cout);
  // parser.displayEndEffectors (std::cout);
  // std::cout << *humanoidRobot << std::endl;
}
