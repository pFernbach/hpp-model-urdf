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
 * \brief Declaration of SRDF Parser.
 */

#ifndef HPP_MODEL_SRDF_PARSER
# define HPP_MODEL_SRDF_PARSER

# include <string>
# include <map>

# include <srdfdom/model.h>

# include <KineoModel/kppJointComponent.h>

# include <hpp/geometry/component/capsule.hh>

# include <hpp/model/capsule-body.hh>
# include <hpp/model/humanoid-robot.hh>

namespace hpp
{
  namespace model
  {
    namespace srdf
    {
      /// \brief Parse an SRDF file to add semantic information
      /// (special configurations, collision pairs) to
      /// an existing hpp::model::HumanoidRobotShPtr.
      class Parser
      {
      public:
	typedef ::srdf::Model::DisabledCollision CollisionPairType;
	typedef std::vector<CollisionPairType> CollisionPairsType;
	typedef std::map<std::string, std::vector<double> > ConfigurationType;

	typedef ::srdf::Model::GroupState SRDFGroupStateType;
	typedef std::vector<SRDFGroupStateType> SRDFGroupStatesType;

	typedef std::vector<CkwsJointShPtr> JointPtrsType;
	typedef std::vector<CkcdObjectShPtr> ObjectPtrsType;

	typedef vectorN HppConfigurationType;
	typedef hpp::model::CapsuleBody BodyType;
	typedef hpp::model::CapsuleBodyShPtr BodyPtrType;
	typedef hpp::geometry::component::Capsule CapsuleType;
	typedef hpp::geometry::component::CapsuleShPtr CapsulePtrType;
	typedef hpp::geometry::component::Segment SegmentType;
	typedef hpp::geometry::component::SegmentShPtr SegmentPtrType;
	typedef hpp::model::HumanoidRobotShPtr RobotPtrType;

	/// \brief Default constructor.
	explicit Parser ();
	/// \brief Destructor.
	virtual ~Parser ();

	/// Display in output stream list of disabled collision pairs.
	void
	displayDisabledCollisionPairs (std::ostream& os);

	/// Display in output stream list of added collision pairs.
	void
	displayAddedCollisionPairs (std::ostream& os);

	/// Compute full configuration of the robot.
	///
	/// This method takes as argument a configuration containing
	/// reference dof values for the actuated joints, places the
	/// robot on the floor at a given height, and computes the
	/// free-flyer joint dof values.
	bool
	computeFullConfiguration (HppConfigurationType& configuration,
				  const bool isRightFootSupporting = true,
				  const double& floorHeight = 0.);

	/// Get reference configuration by name.
	///
	/// The returned configuration is a vector containing all the
	/// dof values. One should make no assumption about the dof
	/// order in the vector.
	///
	/// The only guarantee is that the configuration of the robot
	/// will be correctly set if the returned configuration vector
	/// is given as argument to
	/// hpp::model::Device::hppSetCurrentConfig ().
	HppConfigurationType
	getHppReferenceConfig (const std::string& groupName,
			       const std::string& stateName);

	/// Get reference configuration from SRDF by name.
	///
	/// The returned configuration maps each joint name to a joint
	/// vector. A joint vector contains the joint dof values.
	///
	/// \param groupName name of joints group
	/// \param stateName name of state in group to retrieve
	ConfigurationType
	getReferenceConfig (const std::string& groupName,
			    const std::string& stateName);

	/// \brief Parse an URDF file and add semantic information to
	/// humanoid robot.
	///
	/// The URDF and SRDF file location must use the resource
	/// retriever format.
	///
	/// For instance, the following strings are allowed:
	/// - package://myPackage/robot.urdf
	/// - file:///tmp/robot.urdf
	/// - http://mywebsite.com/robot.urdf
	///
	/// See resource_retriever documentation for more information.
	///
	/// \param resourceName resource name using the
	/// resource_retriever format.
	///
	/// \param robotResourceName URDF resource name
	/// \param semanticResourceName SRDF resource name
	bool
	parse (const std::string& robotResourceName,
	       const std::string& semanticResourceName,
	       RobotPtrType& robot);

	/// \brief Parse an SRDF sent as a stream and add semantic
	/// information to humanoid robot.
	///
	/// \param robotDescription URDF stream
	/// \param semanticDescription SRDF stream
	bool
	parseStream (const std::string& robotDescription,
		     const std::string& semanticDescription,
		     RobotPtrType& robot);

      protected:
	/// \brief Add collision pairs to robot.
	bool
	addCollisionPairs ();

	/// \brief Check if given body pair is disabled.
	bool
	isCollisionPairDisabled (const std::string& bodyName_1,
				 const std::string& bodyName_2);

	/// \brief Check if given body pair has been already been
	/// added.
	bool
	isCollisionPairAdded (const std::string& bodyName_1,
			      const std::string& bodyName_2);

	/// \brief Check if dof vector is consistent with joint.
	bool areDofsInJoint (const std::vector<double>& dofs,
			     const std::string& jointName,
			     std::string& jointType);

      private:
	::urdf::Model urdfModel_;
	::srdf::Model srdfModel_;
	RobotPtrType robot_;
	CollisionPairsType colPairs_;

      }; // class Parser

    } // end of namespace srdf.
  } // end of namespace model.
} // end of namespace hpp.

#endif // HPP_MODEL_SRDF_PARSER
