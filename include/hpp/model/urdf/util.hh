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
#include <hpp/model/rcpdf/parser.hh>

namespace hpp
{
  namespace model
  {
    namespace urdf
    {
      /// \brief Load polyhedron from resource.
      bool
      loadPolyhedronFromResource (const std::string& filename,
				  const ::urdf::Vector3& scale,
				  const CkppPolyhedronShPtr& polyhedron);

      /// \brief Load robot model by name
      ///
      /// \param modelName robot model name
      ///
      /// \param penetration dynamic penetration allowed to validate
      /// direct paths
      ///
      /// \param urdfSuffix suffix for urdf file
      ///
      /// \param srdfSuffix suffix for urdf file
      ///
      /// \param rcpdfSuffix suffix for urdf file
      ///
      /// \note This function works under the assumption that there
      /// exists a ros package ${modelName}_description in which the
      /// directory urdf(resp. srdf and rcpdf) contains the file
      /// ${modelName}${urdfSuffix}.urdf (resp
      /// ${modelName}${srdfSuffix}.srdf and
      /// ${modelName}${rcpdfSuffix}.rcpdf)
      bool loadRobotModel (model::HumanoidRobotShPtr& device,
			   const std::string& modelName,
			   const std::string& urdfSuffix = "",
			   const std::string& srdfSuffix = "",
			   const std::string& rcpdfSuffix = "");

    } // end of namespace urdf.
  } // end of namespace model.
} // end of namespace hpp.

#endif // HPP_MODEL_URDF_PARSER
