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


/// \brief Utility functions.

#ifndef HPP_MODEL_URDF_UTIL
# define HPP_MODEL_URDF_UTIL

#include <hpp/model/urdf/parser.hh>
#include <hpp/model/srdf/parser.hh>

namespace hpp
{
  namespace model
  {
    namespace urdf
    {
      /// Load robot model by name
      ///
      /// \param rootJointType type of root joint among "anchor", "freeflyer",
      /// "planar",
      /// \param modelName robot model name
      /// \param urdfSuffix suffix for urdf file
      /// \param srdfSuffix suffix for srdf file

      /// \note This function reads the following files:
      /// \li
      /// package://${modelName}_description/urdf/${modelName}${urdfSuffix}.urdf
      /// \li
      /// package://${modelName}_description/srdf/${modelName}${srdfSuffix}.srdf
      void loadRobotModel (model::HumanoidRobotPtr_t& device,
			   const std::string& rootJointType,
			   const std::string& modelName,
			   const std::string& urdfSuffix = "",
			   const std::string& srdfSuffix = "");

    } // end of namespace urdf.
  } // end of namespace model.
} // end of namespace hpp.

#endif // HPP_MODEL_URDF_PARSER
