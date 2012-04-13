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
	  robot_ (),
	  colPairs_ ()
      {}

      Parser::~Parser ()
      {}

      void
      Parser::displayDisabledCollisionPairs (std::ostream& os)
      {
	CollisionPairsType disabledColPairs
	  = model_.getDisabledCollisions ();

	os << "Disabled collision pairs list size: "
	   << disabledColPairs.size () << std::endl;

	BOOST_FOREACH (CollisionPairType colPair, disabledColPairs)
	  {
	    os << colPair.first << " " << colPair.second << std::endl;
	  }
      }

      void
      Parser::displayAddedCollisionPairs (std::ostream& os)
      {
	os << "Added collision pairs list size: "
	   << colPairs_.size () << std::endl;

	BOOST_FOREACH (CollisionPairType colPair, colPairs_)
	  {
	    os << colPair.first << " " << colPair.second << std::endl;
	  }
      }

      void
      Parser::addCollisionPairs ()
      {
	// Retrieve joints.
	JointPtrsType joints;
	robot_->getJointVector (joints);

	// Cycle through joints to add collision pairs.
	BOOST_FOREACH (CkwsJointShPtr joint_i, joints)
	  {
	    BodyPtrType body_i;
	    body_i = KIT_DYNAMIC_PTR_CAST (BodyType,
					   joint_i->attachedBody ());
	    if (!body_i)
	      continue;
	    std::string bodyName_i = body_i->name ();

	    BOOST_FOREACH (CkwsJointShPtr joint_j, joints)
	      {
		BodyPtrType body_j;
		body_j = KIT_DYNAMIC_PTR_CAST (BodyType,
					       joint_j->attachedBody ());
		if (!body_j)
		  continue;
		std::string bodyName_j = body_j->name ();

		// Add collision pair after checking it is not
		// disabled, and that it has not been already added.
		if (bodyName_i != bodyName_j
		    && !isCollisionPairDisabled (bodyName_i, bodyName_j)
		    && !isCollisionPairAdded (bodyName_i, bodyName_j))
		  {
		    // Add collision pairs only for bodies that have
		    // geometric objects attached to them.
		    ObjectPtrsType objects_i = body_i->innerObjects ();
		    ObjectPtrsType objects_j = body_j->innerObjects ();
		    if (objects_i.size () > 1 && objects_j.size () > 1)
		      {
			BOOST_FOREACH (CkcdObjectShPtr object, objects_j)
			  body_i->addOuterObject (object);

			colPairs_.push_back
			  (CollisionPairType (bodyName_i, bodyName_j));
		      }
		  }
	      }
	  }
      }

      bool
      Parser::isCollisionPairDisabled (const std::string& bodyName_1,
				       const std::string& bodyName_2)
      {
	// Retrieve collision pairs that will NOT be taken into account.
	CollisionPairsType disabledColPairs
	  = model_.getDisabledCollisions ();

	// Cycle through disabled collision pairs.
	BOOST_FOREACH (CollisionPairType disabledColPair, disabledColPairs)
	  {
	    std::string disabled1 = disabledColPair.first;
	    std::string disabled2 = disabledColPair.second;

	    if ((bodyName_1 == disabled1 && bodyName_2 == disabled2)
		|| (bodyName_1 == disabled2 && bodyName_2 == disabled1))
		return true;
	  }

	return false;
      }

      bool
      Parser::isCollisionPairAdded (const std::string& bodyName_1,
				    const std::string& bodyName_2)
      {
	// Cycle through added collision pairs.
	BOOST_FOREACH (CollisionPairType colPair, colPairs_)
	  {
	    std::string added1 = colPair.first;
	    std::string added2 = colPair.second;

	    if ((bodyName_1 == added1 && bodyName_2 == added2)
		|| (bodyName_1 == added2 && bodyName_2 == added1))
		return true;
	  }

	return false;
      }

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
	colPairs_.clear ();

	// Parse urdf model.
	::urdf::Model urdfModel;
	if (!urdfModel.initString (robotDescription))
	  throw std::runtime_error ("failed to open URDF file."
				    " Is the filename location correct?");

	// Parse srdf model.
	if (!model_.initString (urdfModel, semanticDescription))
	  throw std::runtime_error ("failed to open SRDF file."
				    " Is the filename location correct?");

	// Add collision pairs.
	addCollisionPairs ();
      }

    } // end of namespace urdf.
  } // end of namespace model.
} // end of namespace  hpp.
