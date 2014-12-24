// Copyright (C) 2012, 2013, 2014 CNRS-LAAS
// Authors: Antonio El Khoury, Florent Lamiraux
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

# include <hpp/fcl/BV/OBBRSS.h>
# include <hpp/fcl/BVH/BVH_model.h>

# include <hpp/model/body.hh>
# include <hpp/model/humanoid-robot.hh>
# include <hpp/model/object-factory.hh>

class aiNode;
class aiScene;

namespace hpp
{
  namespace model
  {
    namespace urdf
    {
      /// \brief Parse an URDF file and return a
      /// hpp::model::HumanoidRobotPtr_t.
      class Parser
      {
      public:
	typedef boost::shared_ptr < ::urdf::Link> UrdfLinkPtrType;
	typedef boost::shared_ptr < ::urdf::Joint> UrdfJointPtrType;
	typedef boost::shared_ptr < ::urdf::JointLimits> UrdfJointLimitsPtrType;
	typedef boost::shared_ptr <const ::urdf::Link> UrdfLinkConstPtrType;
	typedef boost::shared_ptr <const ::urdf::Joint> UrdfJointConstPtrType;

	typedef DevicePtr_t RobotPtrType;
	typedef Joint JointType;
	typedef Body BodyType;
	typedef fcl::BVHModel< fcl::OBBRSS > PolyhedronType;
	typedef boost::shared_ptr <PolyhedronType> PolyhedronPtrType;

	typedef Transform3f MatrixHomogeneousType;

	/// \brief Map of abstract robot dynamics compatible joints.
	typedef std::map<const std::string, JointPtr_t> MapHppJointType;
	/// \brief Map of URDF joints.
	typedef std::map<std::string, UrdfJointPtrType> MapJointType;

	/// \brief Default constructor.
	///
	/// \param rootJointType type of root joint among "anchor", "freeflyer",
	/// "planar",
	explicit Parser (const std::string& rooJointType,
			 const RobotPtrType& robot);
	/// \brief Destructor.
	virtual ~Parser ();

	/// \brief Parse an URDF file and return a robot.
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
	void parse (const std::string& resourceName);

	/// Parse a ROS parameter containing a urdf robot description
	/// \param parameterName name of the ROS parameter
	void parseFromParameter (const std::string& parameterName);

	/// \brief Build the robot from the urdf description
	void buildRobot ();

	/// \brief Set special joints in robot.
	void setSpecialJoints ();
	/// \brief Fill gaze.
	void fillGaze ();

      private:
	/// \brief Retrieve joint name attached to a particular link.
	void findSpecialJoint (const std::string& linkName,
			       std::string& jointName);

	/// \brief Find special joints using REP 120.
	///
	/// This is not direct as abstract-robot-dynamics needs
	/// joints where as REP 120 deals with frames/robot links.
	/// We have to use the REP naming standard to identify the
	/// links and then retrieve the attached joints name.
	void findSpecialJoints ();

	/// Create root joint of robot
	void createRootJoint (const std::string& name,
			      const MatrixHomogeneousType& mat,
			      DevicePtr_t robot);

	/// \brief Parse URDF model and get joints.
	///
	/// Each joint in the URDF model is used to build the
	/// corresponding hpp::model::JointPtr_t object.
	void parseJoints ();

	/// \brief Connect recursively joints to their children.
	void connectJoints (const JointPtr_t& rootJoint);

	/// \brief Parse bodies and add them to joints.
	void addBodiesToJoints();

	/// \brief compute body absolute position.
	///
	/// \param link link for which absolute position is computed
	/// \param pose pose in local from, i.e origin of visual or
	/// collision node
	MatrixHomogeneousType computeBodyAbsolutePosition
	(const UrdfLinkConstPtrType& link, const ::urdf::Pose& pose);

	/// \brief Load polyhedron from resource.
	void loadPolyhedronFromResource
	(const std::string& filename, const ::urdf::Vector3& scale,
	 const Parser::PolyhedronPtrType& polyhedron);

	void meshFromAssimpScene (const std::string& name,
				  const ::urdf::Vector3& scale,
				  const aiScene* scene,
				  const PolyhedronPtrType& mesh);

	void buildMesh (const ::urdf::Vector3& scale,
			const aiScene* scene,
			const aiNode* node,
			std::vector<unsigned>& subMeshIndexes,
			const PolyhedronPtrType& mesh);

	/// \brief Add solid component to body.
	///
	/// The visual and collision geometries attached to the link
	/// are used to create the appropriate FCL geometry.
	void addSolidComponentToJoint (const UrdfLinkConstPtrType& link,
				       const JointPtr_t& joint);

	// returns, in a vector, the children of a joint, or a
	// subchildren if no children si present in the actuated
	// joints map.
	std::vector<std::string> getChildrenJoint
	(const std::string& jointName);

	void getChildrenJoint (const std::string& jointName,
			       std::vector<std::string>& result);

	/// Create free-flyer joints and add them to joints map.
	/// If robot is provided, set root joint.
	void createFreeflyerJoint (const std::string& name,
				   const MatrixHomogeneousType& mat,
				   DevicePtr_t robot = DevicePtr_t ());

	/// Create two translations and one rotation in horizontal plane.
	void createPlanarJoint (const std::string& name,
				const MatrixHomogeneousType& mat,
				DevicePtr_t robot);

	/// \brief Create rotation joint and add it to joints map.
	JointPtr_t createRotationJoint
	(const std::string& name, const MatrixHomogeneousType& mat,
	 const MatrixHomogeneousType& urdfLinkInJoint,
	 const UrdfJointLimitsPtrType& limits);

	/// \brief Create continuous joint and add it to joints map.
	JointPtr_t createContinuousJoint
	(const std::string& name, const MatrixHomogeneousType& urdfLinkInJoint,
	 const MatrixHomogeneousType& mat);

	/// \brief Create translation joint and add it to joints map.
	JointPtr_t createTranslationJoint
	(const std::string& name, const MatrixHomogeneousType& mat,
	 const MatrixHomogeneousType& urdfLinkInJoint,
	 const UrdfJointLimitsPtrType& limits);

	/// \brief Create anchor joint and add it to joints map.
	JointPtr_t createAnchorJoint (const std::string& name,
					const MatrixHomogeneousType& mat);

	/// \brief Get joint by looking for string in joints map
	/// attribute.
	JointPtr_t findJoint (const std::string& jointName);

	/// \brief Convert URDF pose to MatrixHomogeneousType transformation.
	MatrixHomogeneousType poseToMatrix (::urdf::Pose p);

	/// \brief Get joint position in given reference frame.
	MatrixHomogeneousType getPoseInReferenceFrame
	(const std::string& referenceJointName,
	 const std::string& currentJointName);

	::urdf::Model model_;
	const RobotPtrType robot_;
	JointPtr_t rootJoint_;
	MapHppJointType jointsMap_;
	std::string rootJointType_;
	/// \brief Special joints names.
	/// \{
	/// Name of the root joint (holding the first link)
	std::string rootJointName_;
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
	std::vector <fcl::Vec3f> vertices_;
	std::vector <fcl::Triangle> triangles_;
	ObjectFactory objectFactory_;
      }; // class Parser
    } // end of namespace urdf.
  } // end of namespace model.
} // end of namespace hpp.

#endif // HPP_MODEL_URDF_PARSER
