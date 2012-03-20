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

#include "hpp/model/urdf/parser.hh"

using boost::test_tools::output_test_stream;

BOOST_AUTO_TEST_CASE (display_robot)
{
  // Validate Kineo license.
  if (!CkppLicense::initialize ())
    {
      std::cout << "Failed to validate Kineo license." << std::endl;
      return;
    }
  
  hpp::model::urdf::Parser parser; 
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
  humanoidRobot = parser.parse ("/home/aelkhour/profiles/kitelab-2.06-i686-linux-ubuntu-10.04/src/unstable/ros/stacks/hrp2/hrp2_14_description/urdf/hrp2.urdf", "base_footprint_joint");

  BOOST_CHECK_EQUAL (!!humanoidRobot, 0);
}
