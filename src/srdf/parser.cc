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
	  = srdfModel_.getDisabledCollisions ();

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
	  = srdfModel_.getDisabledCollisions ();

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
	  throw std::runtime_error ("No pointer to waist.");
	if (!ankle)
	  throw std::runtime_error ("No pointer to suppporting ankle.");
	if (!sole)
	  throw std::runtime_error ("No pointer to suppporting sole.");

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

	    	BOOST_FOREACH (double dof, dofs)
	    	  {
	    	    hppConfig[i] = dof;
	    	    ++i;
	    	  }
	      }
	  }

	// Use the actuated joints dof values to compute the
	// free-flyer joint dof values.
	// We make for now the strong assumption that the floor is
	// flat and at a null height.
	computeFullConfiguration (hppConfig, true, 0.);

	// Check that the configuration has been correctly filled.
	if (i != robot_->numberDof ())
	  throw std::runtime_error
	    ("Incorrect number of dofs in configuration.");
	else
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
	throw std::runtime_error ("Reference configuration not found.");
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
	colPairs_.clear ();

	// Parse urdf model.
	if (!urdfModel_.initString (robotDescription))
	  throw std::runtime_error ("failed to open URDF file."
				    " Is the filename location correct?");

	// Parse srdf model.
	if (!srdfModel_.initString (urdfModel_, semanticDescription))
	  throw std::runtime_error ("failed to open SRDF file."
				    " Is the filename location correct?");

	// Add collision pairs.
	addCollisionPairs ();
      }

    } // end of namespace srdf.
  } // end of namespace model.
} // end of namespace  hpp.
