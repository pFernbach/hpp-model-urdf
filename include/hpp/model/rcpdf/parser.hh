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
 * \brief Declaration of RCPDF Parser.
 */

#ifndef HPP_MODEL_RCPDF_PARSER
# define HPP_MODEL_RCPDF_PARSER

# include <string>
# include <map>

# include <rcpdf/parser.hh>

# include <hpp/model/humanoid-robot.hh>

namespace hpp
{
  namespace model
  {
    namespace rcpdf
    {
      /// \brief Parse an RCPDF file to add contact points information
      /// (hands, feet, etc) to an existing
      /// hpp::model::HumanoidRobotShPtr.
      class Parser
      {
      public:
	typedef boost::shared_ptr<rcpdf_interface::ModelInterface>
	RcpdfModelPtrType;
	typedef boost::shared_ptr<rcpdf_interface::Contact> ContactPtrType;
	typedef std::vector<ContactPtrType> ContactPtrsType; 
	typedef boost::shared_ptr<rcpdf_interface::Geometry> GeometryPtrType;
	typedef std::pair<double, double> SoleDimensionsType;

	typedef CjrlFoot* FootPtrType;
	typedef hpp::model::HumanoidRobotShPtr RobotPtrType;

	/// \brief Default constructor.
	explicit Parser ();
	/// \brief Destructor.
	virtual ~Parser ();

	/// \brief Parse an RCPDF file and add contact points
	/// information to humanoid robot.
	///
	/// The RCPDF file location must use the resource retriever
	/// format.
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
	/// \param contactsResourceName RCPDF resource name
	void
	parse (const std::string& contactsResourceName,
	       RobotPtrType& robot);

	/// \brief Parse an RCPDF sent as a stream and add contact
	/// information to humanoid robot.
	///
	/// \param contactsDescription RCPDF stream
	void
	parseStream (const std::string& contactsDescription,
		     RobotPtrType& robot);

      protected:
	/// \brief Find contact by link name.
	ContactPtrType
	findContact (const std::string& linkName);

	/// \brief Compute sole size from contact points.
	SoleDimensionsType
	computeSoleDimensions (const bool isRightSole);

	/// \brief Set feet size by reading the feet contact points.
	void
	setFeetSize ();

      private:
	RcpdfModelPtrType rcpdfModel_;
	RobotPtrType robot_;

      }; // class Parser

    } // end of namespace rcpdf.
  } // end of namespace model.
} // end of namespace hpp.

#endif // HPP_MODEL_RCPDF_PARSER
