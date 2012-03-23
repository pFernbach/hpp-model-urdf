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

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(CXX_DISABLE_WERROR TRUE)
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/cpack.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/ros.cmake)
INCLUDE(cmake/kineo.cmake)

SET(PROJECT_NAME hpp-model-urdf)
SET(PROJECT_DESCRIPTION "Implementation of a parser for hpp-model")
SET(PROJECT_URL "No webpage yet...")

SETUP_PROJECT()

# Profiling flags when building with DEBUG
#-----------------------------------------
IF(CMAKE_BUILD_TYPE MATCHES "DEBUG")
  IF(CMAKE_COMPILER_IS_GNUCXX)
    SET (${PROJECT_NAME}_CXXFLAGS "-ggdb3 ${${PROJECT_NAME}_CXXFLAGS}")
  ENDIF(CMAKE_COMPILER_IS_GNUCXX)
ENDIF(CMAKE_BUILD_TYPE MATCHES "DEBUG")

# Search for Boost.
# Boost.Test is used by the test suite.
SEARCH_FOR_BOOST()

# Search for dependecies.
ADD_REQUIRED_DEPENDENCY("assimp")
ADD_REQUIRED_DEPENDENCY("KineoController")
ADD_REQUIRED_DEPENDENCY("hpp-model >= 2.1")
ADD_ROSPACK_DEPENDENCY("urdf_parser")
ADD_ROSPACK_DEPENDENCY("jrl_dynamics_urdf")

# Set header files.
SET(${PROJECT_NAME}_HEADERS
  include/hpp/model/urdf/parser.hh
  include/hpp/model/urdf/polyhedron-loader.hh
  )

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(tests)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
