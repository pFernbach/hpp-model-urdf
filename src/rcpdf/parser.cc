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
 * \file src/rcpdf/parser.cc
 *
 * \brief Implementation of RCPDF Parser for hpp-model.
 */

#include <boost/filesystem/fstream.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>

#include <resource_retriever/retriever.h>

#include <hpp/util/debug.hh>

#include <hpp/model/types.hh>

#include "hpp/model/rcpdf/parser.hh"

namespace hpp
{
  namespace model
  {
    namespace rcpdf
    {
      Parser::Parser ()
	: rcpdfModel_ (),
	  robot_ ()
      {}

      Parser::~Parser ()
      {}

      Parser::ContactPtrType
      Parser::findContact (const std::string& linkName)
      {
	ContactPtrsType contacts = rcpdfModel_->contacts_;

	BOOST_FOREACH (ContactPtrType contact, contacts)
	  {
	    if (contact->link_ == linkName)
	      return contact;
	  }

	return ContactPtrType (); 
      }

      Parser::SoleDimensionsType
      Parser::computeSoleDimensions (const bool isRightSole)
      {
	SoleDimensionsType soleDimensions;

	ContactPtrType contact = isRightSole ?
	  findContact ("r_sole") : findContact ("l_sole");
	if (!contact)
	  {
	    hppDout (error, "No contact found for link in RCPDF");
	    soleDimensions.first = 0;
	    soleDimensions.second = 0;
	    return soleDimensions;
	  }
	
	GeometryPtrType geometry = contact->geometry_;
	if (!geometry)
	  {
	    hppDout (error, "No geometry found for contact in RCPDF");
	    soleDimensions.first = 0;
	    soleDimensions.second = 0;
	    return soleDimensions;
	  }

	// FIXME: For now sole geometry can be only of box type.
	if (!geometry->type == ::urdf::Geometry::BOX)
	  {
	    hppDout (error, "Cannot handle this geometry type for now");
	    soleDimensions.first = 0;
	    soleDimensions.second = 0;
	    return soleDimensions;
	  }

	boost::shared_ptr< ::urdf::Box> box
	   = dynamic_pointer_cast< ::urdf::Box> (geometry);
	
	soleDimensions.first = box->dim.x;
	soleDimensions.second = box->dim.y;

	return soleDimensions;
      }

      bool
      Parser::setFeetSize ()
      {
	// Get feet in robot.
	FootPtrType rightFoot = robot_->rightFoot ();
	FootPtrType leftFoot = robot_->leftFoot ();

	if (!rightFoot) {
	  hppDout (notice, "No right foot found.");
	}
	if (!leftFoot) {
	  hppDout (notice, "No left foot found.");
	}
	
	// Compute sole sizes from contact points in feet.
	SoleDimensionsType rightSoleDimensions = computeSoleDimensions (true);
	SoleDimensionsType leftSoleDimensions = computeSoleDimensions (false);

	// Set sole size in robot feet.
	rightFoot->setSoleSize (rightSoleDimensions.first,
				rightSoleDimensions.second);
	leftFoot->setSoleSize (leftSoleDimensions.first,
			       leftSoleDimensions.second);

	return true;
      }

      bool
      Parser::parse (const std::string& contactsResourceName,
		     Parser::RobotPtrType& robot)
      {
	resource_retriever::Retriever resourceRetriever;

	resource_retriever::MemoryResource contactsResource =
	  resourceRetriever.get(contactsResourceName);
	std::string contactsDescription;
	contactsDescription.resize(contactsResource.size);
	for (unsigned i = 0; i < contactsResource.size; ++i)
	  contactsDescription[i] = contactsResource.data.get()[i];

	return parseStream (contactsDescription, robot);
      }

      bool
      Parser::parseStream (const std::string& contactsDescription,
			   Parser::RobotPtrType& robot)
      {
	// Reset the attributes to avoid problems when loading
	// multiple robots using the same object.
	rcpdfModel_.reset ();
	robot_ = robot;

	// Parse rcpdf model.
	rcpdfModel_ = ::rcpdf::parseRCPDF (contactsDescription);
	if (!rcpdfModel_)
	  {
	    hppDout (error, "Failed to open RCPDF file."
		     << " Is the filename location correct?");
	    return false;
	  }

	return setFeetSize ();
      }

    } // end of namespace rcpdf.
  } // end of namespace model.
} // end of namespace  hpp.
