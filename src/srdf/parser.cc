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
#include <ros/node_handle.h>

#include <urdf/model.h>

#include <hpp/util/assertion.hh>
#include <hpp/util/debug.hh>
#include <hpp/util/timer.hh>
#include <hpp/model/collision-object.hh>
#include <hpp/model/joint.hh>

#include <hpp/model/srdf/parser.hh>

namespace hpp
{
  namespace model
  {
    namespace srdf
    {
      namespace {
        HPP_DEFINE_TIMECOUNTER(is_collision_pair_disabled);
      }

      Parser::Parser (urdf::Parser* parser)
	: urdfParser_ (parser), srdfModel_ (), robot_ ()
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

      void Parser::sortCollisionPairs ()
      {
        hppStartBenchmark (sort_collision_pairs);
	// Retrieve collision pairs that will NOT be taken into account.
	const CollisionPairsType& disabledColPairs
	  = srdfModel_.getDisabledCollisionPairs ();
        sortedDisabledCollisions_.resize (disabledColPairs.size());

        std::partial_sort_copy (disabledColPairs.begin (), disabledColPairs.end (),
            sortedDisabledCollisions_.begin (), sortedDisabledCollisions_.end (),
            disabledCollisionComp_);
        hppStopBenchmark (sort_collision_pairs);
        hppDisplayBenchmark (sort_collision_pairs);
      }

      void Parser::addCollisionPairs ()
      {
        hppStartBenchmark (add_collision_pairs);
        typedef urdf::Parser::MapHppJointType MapHppJointType;
        MapHppJointType& jmap = urdfParser_->jointsMap_;

        sortCollisionPairs ();

	// Cycle through all joint pairs
	for (MapHppJointType::const_iterator it1 = jmap.begin ();
	     it1 != jmap.end (); it1++) {
	  JointPtr_t joint1 = it1->second;
	  hppDout (info, "Cycling through joint " << joint1->name ()
		   << ": " << joint1);
	  Body* body1 = joint1->linkedBody ();
	  if (!body1) {
	    hppDout (notice, "Joint " + joint1->name () <<
		     " has no hpp::model::Body.");
	  } else {
	    hppDout (info, "body " << body1);
	    std::string bodyName1 = body1->name ();
	    for (MapHppJointType::const_iterator it2 = jmap.begin ();
		 it2 != it1; it2++) {
	      JointPtr_t joint2 = it2->second;
	      hppDout (info, "  Cycling through joint " << joint2->name ());
	      Body* body2 = joint2->linkedBody ();
	      if (!body2) {
		hppDout (notice, "Joint " + joint2->name () <<
			 " has no hpp::model::Body.");
	      } else {
		std::string bodyName2 = body2->name ();
		if (!isCollisionPairDisabled (
                      removePrefix (bodyName1), 
                      removePrefix (bodyName2))) {
		  hppDout (info, "Handling pair: ("  << bodyName1 << ","
			   << bodyName2 << ")");

		  robot_->addCollisionPairs (joint1, joint2, COLLISION);
		  robot_->addCollisionPairs (joint1, joint2, DISTANCE);
		}
	      }
	    }
	  }
	}
        hppStopBenchmark (add_collision_pairs);
        hppDisplayBenchmark (add_collision_pairs);
      }

      bool
      Parser::isCollisionPairDisabled (const std::string& bodyName_1,
				       const std::string& bodyName_2)
      {
        HPP_START_TIMECOUNTER(is_collision_pair_disabled);
        DisabledCollision dc;
        dc.link1_ = bodyName_1;
        dc.link2_ = bodyName_2;
        if (std::binary_search (sortedDisabledCollisions_.begin (),
              sortedDisabledCollisions_.end (), dc, disabledCollisionComp_)) {
          HPP_STOP_TIMECOUNTER(is_collision_pair_disabled);
          return true;
        }
        dc.link2_ = bodyName_1;
        dc.link1_ = bodyName_2;
        bool ret = std::binary_search (sortedDisabledCollisions_.begin (),
              sortedDisabledCollisions_.end (), dc, disabledCollisionComp_);
        HPP_STOP_TIMECOUNTER(is_collision_pair_disabled);
        return ret;
      }

      bool
      Parser::areDofsInJoint (const std::vector<double>& dofs,
			      const std::string& jointName,
			      std::string& jointType)
      {
	switch (urdfParser_->model_.joints_[jointName]->type)
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
      Parser::parse (const std::string& semanticResourceName,
		     Parser::RobotPtrType robot)
      {
	resource_retriever::Retriever resourceRetriever;

	resource_retriever::MemoryResource semanticResource =
	  resourceRetriever.get(semanticResourceName);
	std::string semanticDescription;
	semanticDescription.resize(semanticResource.size);
	for (unsigned i = 0; i < semanticResource.size; ++i)
	  semanticDescription[i] = semanticResource.data.get()[i];

	// Reset the attributes to avoid problems when loading
	// multiple robots using the same object.
	srdfModel_.clear ();
	robot_ = robot;

	// Parse srdf model.
	if (!srdfModel_.initString (urdfParser_->model_, semanticDescription))
	  {
	    throw std::runtime_error ("Failed to open SRDF file:\n"
				      + semanticDescription);
	  }
	processSemanticDescription ();
      }

      void Parser::parseFromParameter (const std::string& srdfParameterName,
				       RobotPtrType robot)
      {
	// Reset the attributes to avoid problems when loading
	// multiple robots using the same object.
	srdfModel_.clear ();
	robot_ = robot;

	// Parse srdf model. srdf::Model does not support direct parameter
	// reading. We need to load the parameter value in a string
	ros::NodeHandle nh;
	std::string semanticDescription;
	if (nh.getParam (srdfParameterName, semanticDescription)) {
	  if (srdfModel_.initString (urdfParser_->model_, semanticDescription)) {
	    processSemanticDescription ();
	  } else {
	    throw std::runtime_error ("Failed to parse ROS parameter "+
				      srdfParameterName);
	  }
	} else {
	  throw std::runtime_error ("Failed to read ROS parameter "+
				    srdfParameterName);
	}
      }

      void Parser::processSemanticDescription ()
      {
	// Add collision pairs.
	addCollisionPairs ();
        HPP_DISPLAY_TIMECOUNTER(is_collision_pair_disabled);
        HPP_RESET_TIMECOUNTER(is_collision_pair_disabled);
      }
    } // end of namespace srdf.
  } // end of namespace model.
} // end of namespace  hpp.
