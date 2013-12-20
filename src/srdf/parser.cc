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

#include <sstream>
#include <boost/foreach.hpp>
#include <resource_retriever/retriever.h>

#include <urdf/model.h>

#include <jrl/mal/matrixabstractlayer.hh>
#include <hpp/util/assertion.hh>
#include <hpp/util/debug.hh>
#include <hpp/model/collision-object.hh>
#include <hpp/model/joint.hh>

#include <hpp/model/srdf/parser.hh>

namespace hpp
{
  namespace model
  {
    namespace srdf
    {
      Parser::Parser ()
	: urdfModel_ (),
	  srdfModel_ (),
	  robot_ ()
      {}

      Parser::~Parser ()
      {}

      void
      Parser::displayDisabledCollisionPairs (std::ostream& os)
      {
	CollisionPairsType disabledColPairs
	  = srdfModel_.getDisabledCollisionPairs ();

	os << "Disabled collision pairs list size: "
	   << disabledColPairs.size () << std::endl;

	BOOST_FOREACH (CollisionPairType colPair, disabledColPairs)
	  {
	    os << colPair.link1_ << " " << colPair.link2_ << std::endl;
	  }
      }

      void Parser::addCollisionPairs ()
      {
	JointVector_t joints = robot_->getJointVector ();

	// Cycle through all joint pairs
	for (JointVector_t::iterator it1 = joints.begin ();
	     it1 != joints.end (); it1++) {
	  JointPtr_t joint1 = *it1;
	  hppDout (info, "Cycling through joint " << joint1->name ()
		   << ": " << joint1);
	  Body* body1 = joint1->linkedBody ();
	  if (!body1) {
	    hppDout (notice, "Joint " + joint1->name () <<
		     " has no hpp::model::Body.");
	  } else {
	    hppDout (info, "body " << body1);
	    std::string bodyName1 = body1->name ();
	    for (JointVector_t::iterator it2 = joints.begin ();
		 it2 != it1; it2++) {
	      JointPtr_t joint2 = *it2;
	      hppDout (info, "  Cycling through joint " << joint2->name ());
	      Body* body2 = joint2->linkedBody ();
	      if (!body2) {
		hppDout (notice, "Joint " + joint2->name () <<
			 " has no hpp::model::Body.");
	      } else {
		std::string bodyName2 = body2->name ();
		if (!isCollisionPairDisabled (bodyName1, bodyName2)) {
		  hppDout (info, "Handling pair: ("  << bodyName1 << ","
			   << bodyName2 << ")");
		  // Add each inner object of body 1 as outer object of body 2
		  const ObjectVector_t& collisionObjects =
		    body1->innerObjects (COLLISION);
		  hppDout (info, "Number of collision objects in body "
			   << bodyName1 << ": " << collisionObjects.size ());
		  for (ObjectVector_t::const_iterator itObj1 =
			 collisionObjects.begin ();
		       itObj1 != collisionObjects.end (); itObj1++) {
		    body2->addOuterObject (*itObj1, true, false);
		    hppDout (info, "Adding object " << (*itObj1)->name ()
			     << " to body " << body2->name ()
			     << " for collision");
		  }
		  const ObjectVector_t& distanceObjects =
		    body1->innerObjects (DISTANCE);
		  hppDout (info, "Number of distance objects in body "
			   << bodyName1 << ": " << distanceObjects.size ());
		  for (ObjectVector_t::const_iterator itObj1 =
			 distanceObjects.begin ();
		       itObj1 != distanceObjects.end (); itObj1++) {
		    body2->addOuterObject (*itObj1, false, true);
		    hppDout (info, "Adding object " << (*itObj1)->name ()
			     << " to body " << body2->name ()
			     << " for distance");
		  }
		}
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
	  = srdfModel_.getDisabledCollisionPairs ();

	// Cycle through disabled collision pairs.
	BOOST_FOREACH (CollisionPairType disabledColPair, disabledColPairs)
	  {
	    std::string disabled1 = disabledColPair.link1_;
	    std::string disabled2 = disabledColPair.link2_;

	    if ((bodyName_1 == disabled1 && bodyName_2 == disabled2)
		|| (bodyName_1 == disabled2 && bodyName_2 == disabled1))
		return true;
	  }

	return false;
      }

      bool
      Parser::areDofsInJoint (const std::vector<double>& dofs,
			      const std::string& jointName,
			      std::string& jointType)
      {
	switch (urdfModel_.joints_[jointName]->type)
	  {
	  case ::urdf::Joint::REVOLUTE:
	    if (dofs.size () != 1)
	      {
		jointType = "revolute";
		return false;
	      }
	    break;
	  case ::urdf::Joint::CONTINUOUS:
	    if (dofs.size () != 1)
	      {
		jointType = "continous";
		return false;
	      }
	    break;
	  case ::urdf::Joint::PRISMATIC:
	    if (dofs.size () != 1)
	      {
		jointType = "prismatic";
		return false;
	      }
	    break;
	  case ::urdf::Joint::FLOATING:
	    if (dofs.size () != 6)
	      {
		jointType = "floating";
		return false;
	      }
	    break;
	  case ::urdf::Joint::PLANAR:
	    if (dofs.size () != 3)
	      {
		jointType = "planar";
		return false;
	      }
	    break;
	  case ::urdf::Joint::FIXED:
	    if (dofs.size () != 0)
	      {
		jointType = "fixed";
		return false;
	      }
	  default:
	    HPP_ASSERT (0 && "Unknown type of joint.");
	    break;
	  }

	return true;
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
	urdfModel_.clear ();
	srdfModel_.clear ();
	robot_ = robot;

	// Parse urdf model.
	if (!urdfModel_.initString (robotDescription))
	  {
	    throw std::runtime_error ("Failed to open URDF file:\n"+
				      robotDescription);
	  }

	// Parse srdf model.
	if (!srdfModel_.initString (urdfModel_, semanticDescription))
	  {
	    throw std::runtime_error ("Failed to open SRDF file:\n"
				      + semanticDescription);
	  }

	// Add collision pairs.
	addCollisionPairs ();
      }
    } // end of namespace srdf.
  } // end of namespace model.
} // end of namespace  hpp.
