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

# include <KineoUtility/kitMat4.h>

# include <jrl/dynamics/robotdynamicsimpl.hh>
# include <jrl/dynamics/urdf/parser.hh>

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
	typedef std::map<const std::string, hpp::model::JointShPtr> MapHppJoint;

	/// \brief Default constructor.
	explicit Parser ();
	/// \brief Destructor.
	virtual ~Parser ();

	void displayActuatedJoints(std::ostream &os);

	void setSpecificities();

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
	HumanoidRobotShPtr
	parse (const std::string& resourceName,
	       const std::string& rootJointName);

      private:
	void createFreeFlyer(const std::string& inName, const CkitMat4& inMat);
	void createRotation(const std::string& inName, const CkitMat4& inMat);
	void createTranslation(const std::string& inName,
			       const CkitMat4& inMat);
	void createAnchor(const std::string& inName, const CkitMat4& inMat);
	void setRootJoint(const std::string& inName);
	void addChildJoint(const std::string& inParent,
			   const std::string& inChild);
	void setActuatedJoints();
	void setEndEffectors();
	void setFreeFlyerBounds();
	void setGaze(const std::string& inJointName);
	void setWaist(const std::string& inJointName);
	void setChest(const std::string& inJointName);
	void setHand(CimplObjectFactory* objFactory,
		     const std::string& JointName,
		     int side);
	void setFoot(CimplObjectFactory* objFactory,
		     const std::string& JointName,
		     int side);

	CkitMat4 fillMat4(double a00, double a01, double a02, double a03,
			  double a10, double a11, double a12, double a13,
			  double a20, double a21, double a22, double a23,
			  double a30, double a31, double a32, double a33);

	/// \brief Attribute to URDF dyamic robot parser.
	jrl::dynamics::urdf::Parser dynamicParser_;

	/// \brief Constructed robot attribute.
	hpp::model::HumanoidRobotShPtr robot_;

	/// \brief Attribute to latest joint created.
	hpp::model::JointShPtr hppJoint_;

	/// \brief Attribute to hpp joints map.
	MapHppJoint jointMap_;

	/// \brief Attribute to actuated joints.
	std::vector<CjrlJoint*> actuatedJoints_;

      }; // class Parser

    } // end of namespace urdf.
  } // end of namespace model.
} // end of namespace hpp.

#endif // HPP_MODEL_URDF_PARSER
