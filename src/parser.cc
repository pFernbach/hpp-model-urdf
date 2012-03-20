// Copyright (C) 2011, 2012 by Antonio El Khoury.
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
 * \file src/parser.cc
 *
 * \brief Implementation of Parser.
 */

#include <hpp/model/anchor-joint.hh>
#include <hpp/model/freeflyer-joint.hh>
#include <hpp/model/rotation-joint.hh>
#include <hpp/model/translation-joint.hh>

#include "hpp/model/urdf/parser.hh"

namespace hpp
{
  namespace model
  {
    namespace urdf
    {
      Parser::Parser ()
	: dynamicParser_ (),
	  robot_(),
	  hppJoint_ (),
	  jointMap_ (),
	  actuatedJoints_ ()
      {}

      Parser::~Parser ()
      {}

      void
      Parser::createFreeFlyer (const std::string& name, const CkitMat4& mat)
      {
	hppJoint_ = hpp::model::FreeflyerJoint::create (name, mat);
	jointMap_[name] = hppJoint_;
      }

      void
      Parser::createRotation (const std::string& name, const CkitMat4& mat)
      {
	hppJoint_ = hpp::model::RotationJoint::create (name, mat);
	jointMap_[name] = hppJoint_;
      }
      void
      Parser::createTranslation (const std::string& name, const CkitMat4& mat)
      {
	hppJoint_ = hpp::model::TranslationJoint::create (name, mat);
	jointMap_[name] = hppJoint_;
      }

      void
      Parser::createAnchor (const std::string& name, const CkitMat4& mat)
      {
	hppJoint_ = hpp::model::AnchorJoint::create (name, mat);
	jointMap_[name] = hppJoint_;
      }

      void Parser::setRootJoint (const std::string& name)
      {
	robot_->setRootJoint (jointMap_[name]);
      }

      void
      Parser::addChildJoint (const std::string& parent,
			     const std::string& child)
      {
	jointMap_[parent]->addChildJoint (jointMap_[child]);
      }

      CkitMat4
      Parser::fillMat4 (double a00, double a01, double a02, double a03,
			double a10, double a11, double a12, double a13,
			double a20, double a21, double a22, double a23,
			double a30, double a31, double a32, double a33)
      {
	CkitMat4 mat4;
	mat4(0,0) = a00;
	mat4(0,1) = a01;
	mat4(0,2) = a02;
	mat4(0,3) = a03;

	mat4(1,0) = a10;
	mat4(1,1) = a11;
	mat4(1,2) = a12;
	mat4(1,3) = a13;

	mat4(2,0) = a20;
	mat4(2,1) = a21;
	mat4(2,2) = a22;
	mat4(2,3) = a23;

	mat4(3,0) = a30;
	mat4(3,1) = a31;
	mat4(3,2) = a32;
	mat4(3,3) = a33;

	return mat4;
      }

      void Parser::
      setActuatedJoints ()
      {
	// Get actuated joints from dynamic parser and set attribute.
	jrl::dynamics::urdf::Parser::MapJrlJoint mapJrlJoint
	  = dynamicParser_.mapJrlJoint ();
	actuatedJoints_.resize (mapJrlJoint.size ());

	unsigned i = 0;
	for (jrl::dynamics::urdf::Parser::MapJrlJoint::iterator it;
	     it != mapJrlJoint.end ();
	     ++it)
	  {
	    actuatedJoints_[i] = it->second;
	    ++i;
	  }

	robot_->setActuatedJoints(actuatedJoints_);
      }

      void
      Parser::displayFoot (CjrlFoot *aFoot, std::ostream &os)
      {
	vector3d data;

	aFoot->getAnklePositionInLocalFrame (data);
	os << "Ankle position in local frame: " << data << std::endl;

	double lFootWidth=0.0, lFootDepth=0.0;
	aFoot->getSoleSize (lFootDepth, lFootWidth);
	os << "Foot width: " << lFootWidth
	   << " foot depth: " << lFootDepth << std::endl;
      }

      void
      Parser::displayHand (CjrlHand *aHand, std::ostream &os)
      {
	vector3d data;

	aHand->getCenter (data);
	os << "Center: " << data << std::endl;

	aHand->getThumbAxis (data);
	os << "Thumb axis: " << data << std::endl;

	aHand->getForeFingerAxis (data);
	os << "Showing axis: " << data << std::endl;

	aHand->getPalmNormal (data);
	os << "Palm axis: " << data << std::endl;
      }

      void
      Parser::displayEndEffectors (std::ostream &os)
      {
	CjrlHand *aHand;
	aHand = robot_->leftHand ();
	displayHand(aHand,os);
	aHand = robot_->rightHand ();
	displayHand (aHand, os);

	CjrlFoot *aFoot;
	aFoot = robot_->leftFoot ();
	displayFoot (aFoot, os);
	aFoot = robot_->rightFoot ();
	displayFoot (aFoot, os);
      }

      void Parser::setFoot (CimplObjectFactory* objFactory,
			    const std::string& JointName,
			    int side)
      {
	// FIXME: Get foot specificities from urdf or rcpdf.
      }

      void
      Parser::setHand (CimplObjectFactory* objFactory,
		       const std::string& JointName,
		       int side)
      {
	// FIXME: Get hand specificities from urdf or rcpdf.
      }

      void
      Parser::setGaze (const std::string& inJointName)
      {
	// FIXME: Get gaze from urdf.
      }

      void
      Parser::setWaist (const std::string& inJointName)
      {
	CjrlJoint* waistJoint = jointMap_[inJointName]->jrlJoint ();
	robot_->waist (waistJoint);
      }

      void
      Parser::setChest (const std::string& inJointName)
      {
	CjrlJoint* chestJoint = jointMap_[inJointName]->jrlJoint ();
	robot_->chest (chestJoint);
      }

      void
      Parser::setEndEffectors ()
      {
	// FIXME: make robot independant.
	CimplObjectFactory* objFactory = new CimplObjectFactory ();
	std::string JointName;
	// Set feet.
	JointName = "RLEG_JOINT5";
	setFoot (objFactory, JointName, -1);
	JointName = "LLEG_JOINT5";
	setFoot (objFactory, JointName, 1);

	// Set hands.
	JointName = "RARM_JOINT5";
	setHand (objFactory, JointName, -1);
	JointName = "LARM_JOINT5";
	setHand (objFactory, JointName, 1);
      }

      void
      Parser::setSpecificities ()
      {
	// FIXME: make robot independant.
	setEndEffectors ();
	setActuatedJoints ();

	// Set gaze.
	setGaze (std::string ("HEAD_JOINT1"));

	// Set Waist.
	setWaist (std::string ("WAIST"));

	// Set Chest.
	setChest (std::string ("CHEST_JOINT1"));
      }

      void
      Parser::setFreeFlyerBounds ()
      {
	CjrlJoint * jrlRootJoint = robot_->getRootJoint ()->jrlJoint ();
	hpp::model::JointShPtr hppRootJoint = robot_->getRootJoint ();

	/* Translations */
	for(unsigned int i = 0; i < 3; i++) {
	  jrlRootJoint->lowerBound (i,
				    - std::numeric_limits<double>::infinity ());
	  jrlRootJoint->upperBound (i,
				    std::numeric_limits<double>::infinity ());
	}
	/* Rx, Ry */
	for(unsigned int i = 3; i < 5; i++){
	  hppRootJoint->isBounded (i, true);
	  hppRootJoint->lowerBound (i, -M_PI/4);
	  hppRootJoint->upperBound (i, M_PI/4);
	}
	/* Rz */
	jrlRootJoint->lowerBound (5, -std::numeric_limits<double>::infinity ());
	jrlRootJoint->upperBound (5, std::numeric_limits<double>::infinity ());
      }

      void Parser::displayActuatedJoints (std::ostream &os)
      {
	const vectorN currentConfiguration = robot_->currentConfiguration ();
	os << "Actuated joints : " ;
	for (unsigned int i = 0; i < 40; i++)
	  {
	    unsigned int riC = actuatedJoints_[i]->rankInConfiguration ();
	    os << currentConfiguration[riC] << " ";
	  }
	os << std::endl;
      }

      HumanoidRobotShPtr
      Parser::parse (const std::string& filename,
		     const std::string& rootJointName)
      {
	CjrlHumanoidDynamicRobot* humanoidDynamicRobot
	  = dynamicParser_.parse (filename, rootJointName);

	HumanoidRobotShPtr humanoidRobot = HumanoidRobot::create ("robot");
      }

    } // end of namespace urdf.
  } // end of namespace model.
} // end of namespace  hpp.
