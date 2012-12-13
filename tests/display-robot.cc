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

#include <fstream>

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include <KineoModel/kppLicense.h>
#include <KineoModel/kppJointComponent.h>

#include "hpp/model/urdf/util.hh"

using boost::test_tools::output_test_stream;

BOOST_AUTO_TEST_CASE (display_robot)
{
  // Validate Kineo license.
  if (!CkppLicense::initialize ())
    {
      std::cout << "Failed to validate Kineo license." << std::endl;
      return;
    }
  
  hpp::model::HumanoidRobotShPtr humanoidRobot;

  BOOST_CHECK_EQUAL(hpp::model::urdf::loadRobotModel (humanoidRobot,
						      "romeo"),
		    true);

  std::ofstream log ("./display-robot.log");
  log << *(humanoidRobot.get ()) << std::endl;
}
