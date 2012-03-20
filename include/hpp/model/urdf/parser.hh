// Copyright (C) 2012 by Antonio El Khoury.
//
// This file is part of the hpp-model-urdf.
//
// hpp-model-urdf is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// hpp-model-urdf is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with hpp-model-urdf.  If not, see <http://www.gnu.org/licenses/>.


/**
 * \brief Declaration of Parser.
 */

#ifndef HPP_MODEL_URDF_PARSER
# define HPP_MODEL_URDF_PARSER

# include <string>

# include <jrl/dynamics/urdf/parser.hh>

# include <hpp/model/humanoid-robot.hh>

class CjrlHumanoidDynamicRobot;

namespace hpp
{
  namespace model
  {
    namespace urdf
    {
      /// \brief Parse an URDF file and return a
      /// hpp::model::HumanoidRobotShPtr.
      class Parser
      {
      public:
	/// \brief Default constructor.
	explicit Parser ();
	/// \brief Destructor.
	virtual ~Parser ();

	/// \brief Parse an URDF file and return a humanoid robot.
	///
	/// The URDF file location must use the resource retriever format.
	/// For instance, the following strings are allowed:
	/// - package://myPackage/robot.urdf
	/// - file:///tmp/robot.urdf
	/// - http://mywebsite.com/robot.urdf
	///
	/// See resource_retriever documentation for more information.
	///
	/// \param resourceName resource name using the
	/// resource_retriever format.
	HumanoidRobotShPtr
	parse (const std::string& resourceName,
	       const std::string& rootJointName);

      private:
	jrl::dynamics::urdf::Parser dynamicParser_;

      }; // class Parser

    } // end of namespace urdf.
  } // end of namespace model.
} // end of namespace hpp.

#endif // HPP_MODEL_URDF_PARSER
