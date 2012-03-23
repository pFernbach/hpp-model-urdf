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
 * \brief Declaration of polyhedron loader function.
 */

#ifndef HPP_MODEL_URDF_POLYHEDRON_LOADER_HH
# define HPP_MODEL_URDF_POLYHEDRON_LOADER_HH

# include <KineoModel/kppPolyhedron.h>

namespace hpp
{
  namespace model
  {
    namespace urdf
    {
      void
      loadPolyhedronFromResource (const std::string& filename,
				  const CkppPolyhedronShPtr& polyhedron);
    } // end of namespace urdf.
  } // end of namespace model.
} // end of namespace hpp.

#endif // HPP_MODEL_URDF_POLYHEDRON_LOADER_HH
