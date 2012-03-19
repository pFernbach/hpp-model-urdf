# Copyright 2012, 
#
# Antonio El Khoury
#
# JRL/LAAS, CNRS/AIST
#
# This file is part of hpp-model-urdf.
# hpp-model-urdf is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# hpp-model-urdf is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Lesser Public License for more details.
# You should have received a copy of the GNU Lesser General Public License
# along with hpp-model-urdf.  If not, see <http://www.gnu.org/licenses/>.

SET(LIBRARY_NAME ${PROJECT_NAME})

ADD_LIBRARY(${LIBRARY_NAME}
  SHARED
  parser.cc
  )

SET_TARGET_PROPERTIES(${LIBRARY_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})

PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-model)
ROSPACK_USE_DEPENDENCY(${LIBRARY_NAME} urdf_parser)
ROSPACK_USE_DEPENDENCY(${LIBRARY_NAME} jrl_dynamics_parser)

INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION lib)
