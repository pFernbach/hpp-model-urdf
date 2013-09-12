// Copyright (C) 2012 by Antonio El Khoury.
//
// This file is part of the hpp-model-urdf.
//
// hpp-model-urdf is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// hpp-model-urdf is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with hpp-model-urdf.  If not, see <http://www.gnu.org/licenses/>.


/**
 * \brief Declaration of Parser.
 */

#ifndef HPP_MODEL_URDF_PARSER
# define HPP_MODEL_URDF_PARSER

# include <string>
# include <map>

# include <urdf/model.h>

# include <hpp/model/humanoid-robot.hh>

namespace hpp
{
  namespace model
  {
    namespace urdf
    {
      /// \brief Parse an URDF file and return a
      /// hpp::model::HumanoidRobotShPtr.
      class Parser
      {
      public:
	typedef boost::shared_ptr< ::urdf::Link> UrdfLinkPtrType;
	typedef boost::shared_ptr< ::urdf::Joint> UrdfJointPtrType;
	typedef boost::shared_ptr< ::urdf::JointLimits> UrdfJointLimitsPtrType;
	typedef boost::shared_ptr<const ::urdf::Link> UrdfLinkConstPtrType;
	typedef boost::shared_ptr<const ::urdf::Joint> UrdfJointConstPtrType;

	typedef hpp::model::HumanoidRobotShPtr RobotPtrType;
	typedef hpp::model::Joint JointType;
	typedef hpp::model::JointShPtr JointPtrType;
	typedef CkwsKCDBodyAdvanced BodyType;
	typedef CkwsKCDBodyAdvancedShPtr BodyPtrType;
	typedef CjrlHand* HandPtrType;
	typedef CjrlFoot* FootPtrType;

	/// \brief Map of abstract robot dynamics compatible joints.
	typedef std::map<const std::string, JointPtrType> MapHppJointType;
	/// \brief Map of URDF joints.
	typedef std::map<std::string, UrdfJointPtrType> MapJointType;

	/// \brief Default constructor.
	explicit Parser ();
	/// \brief Destructor.
	virtual ~Parser ();

	void displayActuatedJoints(std::ostream &os);
	void displayFoot(CjrlFoot* aFoot,std::ostream& os);
	void displayHand(CjrlHand* aHand,std::ostream& os);
	void displayEndEffectors(std::ostream& os);

	/// \brief Parse an URDF file and return a humanoid robot.
	///
	/// The URDF file location must use the resource retriever format.
	/// For instance, the following strings are allowed:
	/// - package://myPackage/robot.urdf
	/// - file:///tmp/robot.urdf
	/// - http://mywebsite.com/robot.urdf
	///
	/// See resource_retriever documentation for more information.
	///
	/// \param resourceName resource name using the
	/// resource_retriever format.
	RobotPtrType
	parse (const std::string& resourceName);

	/// \brief Parse an URDF sent as a stream and return a
	/// humanoid robot.
	RobotPtrType
	parseStream (const std::string& robotDescription);

      protected:
	/// \brief Retrieve joint name attached to a particular link.
	void
	findSpecialJoint (const std::string& linkName,
			  std::string& jointName);

	/// \brief Find special joints using REP 120.
	///
	/// This is not direct as abstract-robot-dynamics needs
	/// joints where as REP 120 deals with frames/robot links.
	/// We have to use the REP naming standard to identify the
	/// links and then retrieve the attached joints name.
	void
	findSpecialJoints ();

	/// \brief Set special joints in robot.
	void
	setSpecialJoints ();

	/// \brief Parse URDF model and get joints.
	///
	/// Each joint in the URDF model is used to build the
	/// corresponding hpp::model::JointShPtr object.
	bool
	parseJoints ();

	/// \brief Get actuated joints.
	std::vector<CjrlJoint*>
	actuatedJoints();

	/// \brief Connect recursively joints to their children.
	bool
	connectJoints (const JointPtrType& rootJoint);

	/// \brief Parse bodies and add them to joints.
	bool addBodiesToJoints();

	/// \brief compute body absolute position.
	///
	/// \param link link for which absolute position is computed
	/// \param pose pose in local from, i.e origin of visual or
	/// collision node
	CkitMat4
	computeBodyAbsolutePosition (const UrdfLinkConstPtrType& link,
				     const ::urdf::Pose& pose);

	/// \brief Add solid component to body.
	///
	/// The visual and collision geometries attached to the link
	/// are used to create the appropriate Kite solid component.
	bool
	addSolidComponentToJoint (const UrdfLinkConstPtrType& link,
				  const JointPtrType& joint);

	/// \brief Set free-flyer joint bounds for roamdap builders.
	void
	setFreeFlyerBounds ();

	/// \brief Compute hands information.
	void
	computeHandsInformation (MapHppJointType::const_iterator& hand,
				 MapHppJointType::const_iterator& wrist,
				 vector3d& center,
				 vector3d& thumbAxis,
				 vector3d& foreFingerAxis,
				 vector3d& palmNormal) const;

	/// \brief Fill gaze.
	void fillGaze ();

	/// \brief Fill hands and feet.
	void
	fillHandsAndFeet ();

	// returns, in a vector, the children of a joint, or a
	// subchildren if no children si present in the actuated
	// joints map.
	std::vector<std::string>
	getChildrenJoint (const std::string& jointName);

	bool getChildrenJoint (const std::string& jointName,
			       std::vector<std::string>& result);

	/// \brief Create free-flyer joint and add it to joints map.
	JointPtrType
	createFreeflyerJoint (const std::string& name, const CkitMat4& mat);

	/// \brief Create rotation joint and add it to joints map.
	JointPtrType
	createRotationJoint (const std::string& name,
			     const CkitMat4& mat,
			     const UrdfJointLimitsPtrType& limits);

	/// \brief Create continuous joint and add it to joints map.
	JointPtrType
	createContinuousJoint (const std::string& name,
			       const CkitMat4& mat);

	/// \brief Create translation joint and add it to joints map.
	JointPtrType
	createTranslationJoint (const std::string& name,
				const CkitMat4& mat,
				const UrdfJointLimitsPtrType& limits);

	/// \brief Create anchor joint and add it to joints map.
	JointPtrType
	createAnchorJoint (const std::string& name, const CkitMat4& mat);

	/// \brief Get joint by looking for string in joints map
	/// attribute.
	JointPtrType
	findJoint (const std::string& jointName);

	/// \brief Compute ankle position in foot frame.
	vector3d
	computeAnklePositionInLocalFrame
	(MapHppJointType::const_iterator& foot,
	 MapHppJointType::const_iterator& ankle) const;

	/// \brief Convert URDF pose to CkitMat4 transformation.
	CkitMat4
	poseToMatrix (::urdf::Pose p);

	/// \brief Get joint position in given reference frame.
	CkitMat4
	getPoseInReferenceFrame(const std::string& referenceJointName,
				const std::string& currentJointName);

      private:
	::urdf::Model model_;
	RobotPtrType robot_;
	JointPtrType rootJoint_;
	MapHppJointType jointsMap_;
	dynamicsJRLJapan::ObjectFactory factory_;

	/// \brief Special joints names.
	/// \{
	std::string waistJointName_;
	std::string chestJointName_;
	std::string leftWristJointName_;
	std::string rightWristJointName_;
	std::string leftHandJointName_;
	std::string rightHandJointName_;
	std::string leftAnkleJointName_;
	std::string rightAnkleJointName_;
	std::string leftFootJointName_;
	std::string rightFootJointName_;
	std::string gazeJointName_;
	/// \}

      }; // class Parser

    } // end of namespace urdf.
  } // end of namespace model.
} // end of namespace hpp.

#endif // HPP_MODEL_URDF_PARSER
