// Copyright (C) 2011, 2012 by Antonio El Khoury.
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

/**
 * \file src/parser.cc
 *
 * \brief Implementation of Parser.
 */

#include "hpp/model/urdf/parser.hh"

namespace hpp
{
  namespace model
  {
    namespace urdf
    {
      Parser::Parser ()
	: dynamicParser_ ()
      {}
      
      Parser::~Parser ()
      {}

      HumanoidRobotShPtr
      Parser::parse (const std::string& filename,
		     const std::string& rootJointName)
      {
	CjrlHumanoidDynamicRobot* humanoidDynamicRobot
	  = dynamicParser_.parse (filename, rootJointName);

	HumanoidRobotShPtr humanoidRobot = HumanoidRobot::create ("robot");
      }

    } // end of namespace urdf.
  } // end of namespace model.
} // end of namespace  hpp.
