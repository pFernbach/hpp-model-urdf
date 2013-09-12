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

#include <hpp/util/assertion.hh>
#include <hpp/util/debug.hh>

#include <hpp/model/urdf/types.hh>
#include "hpp/model/srdf/parser.hh"

namespace hpp
{
  namespace model
  {
    namespace srdf
    {
      Parser::Parser ()
	: urdfModel_ (),
	  srdfModel_ (),
	  robot_ (),
	  colPairs_ ()
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

      void
      Parser::displayAddedCollisionPairs (std::ostream& os)
      {
	os << "Added collision pairs list size: "
	   << colPairs_.size () << std::endl;

	BOOST_FOREACH (CollisionPairType colPair, colPairs_)
	  {
	    os << colPair.link1_ << " " << colPair.link2_ << std::endl;
	  }
      }

      bool
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

	    // Build distance computation analyses for inner objects.
	    ObjectPtrsType objects_i = body_i->innerObjects ();
	    BOOST_FOREACH (CkcdObjectShPtr object_i, objects_i)
	      {
		// Treat segment case separately for fast distance
		// computation.
		SegmentPtrType segment_i
		  = KIT_DYNAMIC_PTR_CAST (SegmentType, object_i);
		if (segment_i)
		  body_i->addInnerCapsule (segment_i, true);
		body_i->addInnerObject (object_i, false);
	      }

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
		    ObjectPtrsType objects_j = body_j->innerObjects ();
		    for (unsigned i = 0; i < objects_j.size (); ++i)
		      {
			CkcdObjectShPtr object = objects_j[i];
			// Treat segment case separately for fast distance
			// computation.
			SegmentPtrType segment
			  = KIT_DYNAMIC_PTR_CAST (SegmentType, object);
			if (segment)
			  body_i->addOuterCapsule (segment, true);
			body_i->addOuterObject (object, false);
		      }
		    CollisionPairType cpt;
		    cpt.link1_ = bodyName_i;
		    cpt.link2_ = bodyName_j;
		    colPairs_.push_back (cpt);
		  }
	      }
	  }

	return true;
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
      Parser::isCollisionPairAdded (const std::string& bodyName_1,
				    const std::string& bodyName_2)
      {
	// Cycle through added collision pairs.
	BOOST_FOREACH (CollisionPairType colPair, colPairs_)
	  {
	    std::string added1 = colPair.link1_;
	    std::string added2 = colPair.link2_;

	    if ((bodyName_1 == added1 && bodyName_2 == added2)
		|| (bodyName_1 == added2 && bodyName_2 == added1))
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

      bool
      Parser::computeFullConfiguration (HppConfigurationType& configuration,
					const bool isRightFootSupporting,
					const double& floorHeight)
      {
	// Set current robot configuration.
	robot_->hppSetCurrentConfig (configuration);

	CjrlJoint* waist = robot_->waist ();
	CjrlJoint* ankle = isRightFootSupporting ?
	  robot_->rightAnkle() : robot_->leftAnkle();
	CjrlFoot* sole = isRightFootSupporting ?
	  robot_->rightFoot () : robot_->leftFoot ();

	if (!waist)
	  {
	    hppDout (error, "No pointer to waist.");
	    return false;
	  }
	if (!ankle)
	  {
	    hppDout (error, "No pointer to supporting ankle.");
	    return false;
	  }
	if (!sole)
	  {
	    hppDout (error, "No pointer to supporting sole.");
	    return false;
	  }

	// Compute waist transformation in floor given an absolute
	// (right) foot transformation.
	matrix4d waistTInWorld = waist->currentTransformation ();
	matrix4d ankleTInWorld = ankle->currentTransformation ();
	vector3d anklePInSole;
	sole->getAnklePositionInLocalFrame (anklePInSole);
	matrix4d ankleTInFloor;
	ankleTInFloor(0,0) = 1;
	ankleTInFloor(1,1) = 1;
	ankleTInFloor(2,2) = 1;
	ankleTInFloor(3,3) = 1;
	ankleTInFloor(0,3) = anklePInSole[0];
	ankleTInFloor(1,3) = anklePInSole[1];
	ankleTInFloor(2,3) = anklePInSole[2] + floorHeight;

	matrix4d worldTInAnkle;
	MAL_S4x4_INVERSE (ankleTInWorld, worldTInAnkle, double);
	matrix4d waistTInAnkle;
	MAL_S4x4_C_eq_A_by_B (waistTInAnkle, worldTInAnkle, waistTInWorld);
	matrix4d waistTInFloor;
	MAL_S4x4_C_eq_A_by_B (waistTInFloor, ankleTInFloor, waistTInAnkle);

	// Fill free-flyer dof values in configuration vector.
	configuration[0] = 0;
	configuration[1] = 0;
	configuration[2] = waistTInFloor(2,3);
	configuration[3] = atan2 (waistTInFloor(1,2), waistTInFloor(2,2));
	configuration[4] = - asin (waistTInFloor(0,2));
	configuration[5] = atan2 (waistTInFloor(0,1), waistTInFloor(0,0));

	return true;
      }

      Parser::HppConfigurationType
      Parser::getHppReferenceConfig (const std::string& groupName,
				     const std::string& stateName)
      {
	// Retrieve joint name to dof map.
	ConfigurationType jointToDof
	  = getReferenceConfig (groupName, stateName);

	// Retrieve robot joint vector.
	std::vector <CkppJointComponentShPtr> joints;
	robot_->getJointComponentVector (joints);

	// Reserve first 6 dofs for free-floating. Their value will be
	// set later.
	HppConfigurationType hppConfig (robot_->numberDof ());
	unsigned i = 6;

	// Cycle through joint vector and add corresponding dof
	// values to configuration vector.
	BOOST_FOREACH (CkppJointComponentShPtr joint, joints)
	  {
	    ConfigurationType::iterator it = jointToDof.find (joint->name ());

	    if (it != jointToDof.end ())
	      {
	    	std::vector<double> dofs = it->second;

		std::string jointType;
		if (!areDofsInJoint (dofs, joint->name (), jointType))
		  {
		    hppDout (error,
			     "Number of joint dofs ("
			     << dofs.size ()
			     << ") is inconsistent with joint type ("
			     << jointType << ") of "
			     << joint->name ());
		    hppConfig.clear ();
		    return hppConfig;
		  }

	    	BOOST_FOREACH (double dof, dofs)
	    	  {
		    if (i == robot_->numberDof ())
		      {
			hppDout (error,
				 "Incorrect number of dofs in configuration.");
			hppConfig.clear ();
			return hppConfig;
		      }

	    	    hppConfig[i] = dof;
	    	    ++i;
	    	  }
	      }
	    else
	      if (joint->name ()!= "base_joint"
		  && joint->kwsJoint ()->countDofs () != 0)
	      {
		hppDout (error, "Reference dof values for joint "
			 << joint->name () << " not found.");
		hppConfig[i] = 0;
		++i;
	      }
	  }

	// Use the actuated joints dof values to compute the
	// free-flyer joint dof values.
	// We make for now the strong assumption that the floor is
	// flat and at a null height.
	if (!computeFullConfiguration (hppConfig, true, 0.))
	  {
	    hppDout (error, "Could not compute full configuration.");
	    hppConfig.clear ();
	    return hppConfig;
	  }

	return hppConfig;
      }

      Parser::ConfigurationType
      Parser::getReferenceConfig (const std::string& groupName,
				  const std::string& stateName)
      {
	SRDFGroupStatesType groupStates = srdfModel_.getGroupStates ();

	// Find group state by name.
	BOOST_FOREACH (SRDFGroupStateType groupState, groupStates)
	  {
	    if (groupState.group_ == groupName
		&& groupState.name_ == stateName)
	      {
		return groupState.joint_values_;
	      }
	  }

	// Throw error if reference configuration is not found.
	hppDout (error, "Reference configuration " << stateName
		 << " in group " << groupName << " not found.");
	ConfigurationType config;
	return config;
      }

      bool
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

	return parseStream (robotDescription, semanticDescription, robot);
      }

      bool
      Parser::parseStream (const std::string& robotDescription,
			   const std::string& semanticDescription,
			   Parser::RobotPtrType& robot)
      {
	// Reset the attributes to avoid problems when loading
	// multiple robots using the same object.
	urdfModel_.clear ();
	srdfModel_.clear ();
	robot_ = robot;
	colPairs_.clear ();

	// Parse urdf model.
	if (!urdfModel_.initString (robotDescription))
	  {
	    hppDout (error,
		     "Failed to open URDF file."
		     << " Is the filename location correct?");
	    return false;
	  }
	
	// Parse srdf model.
	if (!srdfModel_.initString (urdfModel_, semanticDescription))
	  {
	    hppDout (error, "Failed to open SRDF file."
		     << " Is the filename location correct?");
	    return false;
	  }

	// Add collision pairs.
	if (!addCollisionPairs ())
	  {
	    hppDout (error, "Failed to add collision pairs.");
	    return false;
	  }

	return true;
      }

    } // end of namespace srdf.
  } // end of namespace model.
} // end of namespace  hpp.
