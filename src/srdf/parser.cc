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

/**
 * \file src/urdf/parser.cc
 *
 * \brief Implementation of URDF Parser for hpp-model.
 */

#include <boost/filesystem/fstream.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>

#include <resource_retriever/retriever.h>

#include <urdf/model.h>

#include "hpp/model/srdf/parser.hh"

namespace hpp
{
  namespace model
  {
    namespace srdf
    {
      Parser::Parser ()
	: model_ (),
	  robot_ ()
      {}

      Parser::~Parser ()
      {}

      void
      Parser::parse (const std::string& robotResourceName,
		     const std::string& semanticResourceName,
		     Parser::RobotPtrType& robot)
      {
	resource_retriever::Retriever resourceRetriever;

	resource_retriever::MemoryResource robotResource =
	  resourceRetriever.get(robotResourceName);
	std::string robotDescription;
	robotDescription.resize(robotResource.size);
	for (unsigned i = 0; i < robotResource.size; ++i)
	  robotDescription[i] = robotResource.data.get()[i];

	resource_retriever::MemoryResource semanticResource =
	  resourceRetriever.get(semanticResourceName);
	std::string semanticDescription;
	semanticDescription.resize(semanticResource.size);
	for (unsigned i = 0; i < semanticResource.size; ++i)
	  semanticDescription[i] = semanticResource.data.get()[i];
	
	parseStream (robotDescription, semanticDescription, robot);
      }

      void
      Parser::parseStream (const std::string& robotDescription,
			   const std::string& semanticDescription,
			   Parser::RobotPtrType& robot)
      {
	// Reset the attributes to avoid problems when loading
	// multiple robots using the same object.
	model_.clear ();
	robot_ = robot;

	// Parse urdf model.
	::urdf::Model urdfModel;
	if (!urdfModel.initString (robotDescription))
	  throw std::runtime_error ("failed to open URDF file."
				    " Is the filename location correct?");

	// Parse srdf model.
	if (!model_.initString (urdfModel, semanticDescription))
	  throw std::runtime_error ("failed to open SRDF file."
				    " Is the filename location correct?");
      }

    } // end of namespace urdf.
  } // end of namespace model.
} // end of namespace  hpp.
