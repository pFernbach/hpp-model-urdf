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

//#include <boost/numeric/conversion/bounds.hpp>
#include <limits>

#include <boost/filesystem/fstream.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>

#include <resource_retriever/retriever.h>
#include <assimp/DefaultLogger.hpp>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/IOStream.hpp>
#include <assimp/IOSystem.hpp>

#include <hpp/util/debug.hh>
#include <hpp/util/assertion.hh>
#include <hpp/model/collision-object.hh>
#include <hpp/model/device.hh>
#include <hpp/model/fcl-to-eigen.hh>
#include <hpp/model/joint.hh>
#include <hpp/model/object-factory.hh>
#include <hpp/model/urdf/parser.hh>
#include <hpp/model/urdf/util.hh>

#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/shape/geometric_shapes.h>

namespace fcl {
  HPP_PREDEF_CLASS (CollisionGeometry);
}

namespace hpp
{
  namespace model
  {
    namespace urdf
    {
      class ResourceIOStream : public Assimp::IOStream
      {
      public:
	ResourceIOStream (const resource_retriever::MemoryResource& res)
	  : res_(res)
	  , pos_(res.data.get())
	{}

	~ResourceIOStream()
	{}

	size_t Read (void* buffer, size_t size, size_t count)
	{
	  size_t to_read = size * count;
	  if (pos_ + to_read > res_.data.get() + res_.size)
	    {
	      to_read = res_.size - (pos_ - res_.data.get());
	    }

	  memcpy(buffer, pos_, to_read);
	  pos_ += to_read;

	  return to_read;
	}

	size_t Write (const void*, size_t, size_t) { return 0; }

	aiReturn Seek (size_t offset, aiOrigin origin)
	{
	  uint8_t* new_pos = 0;
	  switch (origin)
	    {
	    case aiOrigin_SET:
	      new_pos = res_.data.get() + offset;
	      break;
	    case aiOrigin_CUR:
	      new_pos = pos_ + offset; // TODO is this right?  can offset really not be negative
	      break;
	    case aiOrigin_END:
	      new_pos = res_.data.get() + res_.size - offset; // TODO is this right?
	      break;
	    default:
	      break;
	    }

	  if (new_pos < res_.data.get() || new_pos > res_.data.get() + res_.size)
	    {
	      return aiReturn_FAILURE;
	    }

	  pos_ = new_pos;
	  return aiReturn_SUCCESS;
	}

	size_t Tell() const
	{
	  return pos_ - res_.data.get();
	}

	size_t FileSize() const
	{
	  return res_.size;
	}

	void Flush() {}

      private:
	resource_retriever::MemoryResource res_;
	uint8_t* pos_;
      };

      class ResourceIOSystem : public Assimp::IOSystem
      {
      public:
	ResourceIOSystem()
	{
	}

	~ResourceIOSystem()
	{
	}

	// Check whether a specific file exists
	bool Exists(const char* file) const
	{
	  // Ugly -- two retrievals where there should be one (Exists + Open)
	  // resource_retriever needs a way of checking for existence
	  // TODO: cache this
	  resource_retriever::MemoryResource res;
	  try
	    {
	      res = retriever_.get(file);
	    }
	  catch (resource_retriever::Exception& e)
	    {
	      hppDout (error, e.what ());
	      return false;
	    }

	  return true;
	}

	// Get the path delimiter character we'd like to see
	char getOsSeparator() const
	{
	  return '/';
	}

	// ... and finally a method to open a custom stream
	Assimp::IOStream* Open(const char* file,
			       const char* hppDebugStatement (mode))
	{
	  HPP_ASSERT (mode == std::string("r") || mode == std::string("rb"));

	  // Ugly -- two retrievals where there should be one (Exists + Open)
	  // resource_retriever needs a way of checking for existence
	  resource_retriever::MemoryResource res;
	  try
	    {
	      res = retriever_.get(file);
	    }
	  catch (resource_retriever::Exception& e)
	    {
	      return 0;
	    }

	  return new ResourceIOStream(res);
	}

	void Close(Assimp::IOStream* stream) { delete stream; }

      private:
	mutable resource_retriever::Retriever retriever_;
      };

      using std::numeric_limits;
      Parser::Parser (const std::string& rootJointType,
                      const RobotPtrType& robot,
                      const JointPtr_t& baseJoint)
  : model_ (),
    robot_ (robot),
    rootJoint_ (),
    baseJoint_ (baseJoint),
    jointsMap_ (),
    rootJointType_ (rootJointType),
    prefix_ (),
    rootJointName_ (),
    chestJointName_ (),
    leftWristJointName_ (),
    rightWristJointName_ (),
    leftHandJointName_ (),
    rightHandJointName_ (),
    leftAnkleJointName_ (),
    rightAnkleJointName_ (),
    leftFootJointName_ (),
    rightFootJointName_ (),
    gazeJointName_ ()
      {
#ifdef HPP_DEBUG
	std::string filename = hpp::debug::getPrefix ("assimp") +
	  "/assimp.log";
	hppDout (notice, filename);
	Assimp::DefaultLogger::create (filename.c_str (),
				       Assimp::Logger::VERBOSE);
#endif
      }

      Parser::~Parser ()
      {}

      namespace
      {
	/// \brief Convert joint orientation to standard
	/// jrl-dynamics accepted orientation.
	///
	/// abstract-robot-dynamics do not contain any information
	/// about around which axis a rotation joint rotates.
	/// On the opposite, it makes the assumption it is around the X
	/// axis. We have to make sure this is the case here.
	///
	/// We use Gram-Schmidt process to compute the rotation matrix.
	///
	/// [1] http://en.wikipedia.org/wiki/Gram%E2%80%93Schmidt_process
	Parser::MatrixHomogeneousType
	normalizeFrameOrientation (Parser::UrdfJointConstPtrType urdfJoint)
	{
	  if (!urdfJoint) {
	    throw std::runtime_error
	      ("Null pointer in normalizeFrameOrientation");
	  }

	  Parser::MatrixHomogeneousType result;
	  result.setIdentity ();

	  vector3_t x (urdfJoint->axis.x,
		       urdfJoint->axis.y,
		       urdfJoint->axis.z);
	  x.normalize ();

	  vector3_t y (0., 0., 0.);
	  vector3_t z (0., 0., 0.);

	  unsigned smallestComponent = 0;
	  for (unsigned i = 0; i < 3; ++i)
	    if (std::fabs(x[i]) < std::fabs(x[smallestComponent]))
	      smallestComponent = i;

	  y[smallestComponent] = 1.;
	  z = x.cross (y);
	  y = z.cross (x);
	  // (x, y, z) is an orthonormal basis.

	  fcl::Matrix3f R;
	  for (unsigned i = 0; i < 3; ++i)
	    {
	      R (i, 0) = x[i];
	      R (i, 1) = y[i];
	      R (i, 2) = z[i];
	    }
	  result.setRotation (R);
	  return result;
	}
      } // end of anonymous namespace.

      void
      Parser::findSpecialJoint (const std::string& repName,
				std::string& jointName)
      {
	UrdfLinkPtrType linkPtr = model_.links_[repName];
	if (linkPtr)
	  {
	    UrdfJointPtrType joint = linkPtr->parent_joint;
	    if (joint)
	      jointName = prependPrefix (joint->name);
	  }
      }

      void
      Parser::findSpecialJoints ()
      {
	rootJointName_ = prependPrefix ("base_joint_SO3");
	findSpecialJoint ("torso", chestJointName_);
	findSpecialJoint ("l_wrist", leftWristJointName_);
	findSpecialJoint ("r_wrist", rightWristJointName_);
	findSpecialJoint ("l_gripper", leftHandJointName_);
	findSpecialJoint ("r_gripper", rightHandJointName_);
	findSpecialJoint ("l_ankle", leftAnkleJointName_);
	findSpecialJoint ("r_ankle", rightAnkleJointName_);
	findSpecialJoint ("l_sole", leftFootJointName_);
	findSpecialJoint ("r_sole", rightFootJointName_);
	findSpecialJoint ("gaze", gazeJointName_);
	//FIXME: we are missing toes in abstract-robot-dynamics for now.
      }

      void
      Parser::setSpecialJoints ()
      {
	HumanoidRobotPtr_t robot = HPP_DYNAMIC_PTR_CAST (HumanoidRobot,
							 robot_);
	if (!robot) {
	  throw std::runtime_error ("Robot is not a humanoid");
	}
	try {
	  robot->waist (findJoint (rootJointName_));
	} catch (const std::exception&) {
	  hppDout (notice, "No waist joint found");
	}
	try {
	  robot->chest (findJoint (chestJointName_));
	} catch (const std::exception&) {
	  hppDout (notice, "No chest joint found");
	}
	try {
	  robot->leftWrist (findJoint (leftWristJointName_));
	} catch (const std::exception&) {
	  hppDout (notice, "No left wrist joint found");
	}
	try {
	  robot->rightWrist (findJoint (rightWristJointName_));
	} catch (const std::exception&) {
	  hppDout (notice, "No right wrist joint found");
	}
	try {
	  robot->leftAnkle (findJoint (leftAnkleJointName_));
	} catch (const std::exception&) {
	  hppDout (notice, "No left ankle joint found");
	}
	try {
	  robot->rightAnkle (findJoint (rightAnkleJointName_));
	} catch (const std::exception&) {
	  hppDout (notice, "No right ankle joint found");
	}
	try {
	  robot->gazeJoint (findJoint (gazeJointName_));
	} catch (const std::exception&) {
	  hppDout (notice, "No gaze joint found");
	}
      }

      void Parser::createRootJoint (const std::string& name,
				    const MatrixHomogeneousType& mat,
				    DevicePtr_t robot)
      {
	if (rootJointType_ == "freeflyer") {
	  createFreeflyerJoint (name, mat, robot);
	  rootJointName_ = name + "_SO3";
	  rootJoint_ = jointsMap_ [rootJointName_];
	} else if (rootJointType_ == "anchor") {
	  createAnchorJoint (name, mat);
	  rootJointName_ = name;
	  rootJoint_ = jointsMap_ [rootJointName_];
          if (baseJoint_) baseJoint_->addChildJoint (rootJoint_);
          else robot->rootJoint (rootJoint_);
	} else if (rootJointType_ == "planar") {
	  createPlanarJoint (name, mat, robot);
	  rootJointName_ = name + "_rz";
	  rootJoint_ = jointsMap_ [rootJointName_];
	} else {
	  throw std::runtime_error ("Root joint should be either, \"anchor\","
				    "\"freeflyer\" of \"planar\"");
        }
      }

      void Parser::parseJoints ()
      {
	// Create free floating joint.
	// FIXME: position set to identity for now.
	MatrixHomogeneousType position;
	position.setIdentity ();
	createRootJoint (prependPrefix ("base_joint"), position, robot_);

	// Iterate through each "true kinematic" joint and create a
	// corresponding hpp::model::Joint.
	for(MapJointType::const_iterator it = model_.joints_.begin();
	    it != model_.joints_.end(); ++it) {
	  position =
	    getPoseInReferenceFrame("base_footprint_joint", it->first);

	  // Normalize orientation if this is an actuated joint.
	  UrdfJointConstPtrType joint = model_.getJoint (it->first);
	  Transform3f urdfLinkInJoint;
	  if (joint->type == ::urdf::Joint::REVOLUTE
	      || joint->type == ::urdf::Joint::CONTINUOUS
	      || joint->type == ::urdf::Joint::PRISMATIC) {
	    Transform3f jointInUrdfLink = normalizeFrameOrientation (joint);
	    urdfLinkInJoint = inverse (jointInUrdfLink);
	    position = position * jointInUrdfLink;
	  }

          std::string jointName = prependPrefix (it->first);
	  switch(it->second->type) {
	  case ::urdf::Joint::UNKNOWN:
	    throw std::runtime_error ("Joint has UNKNOWN type");
	    break;
	  case ::urdf::Joint::REVOLUTE:
	    createRotationJoint (jointName, position, urdfLinkInJoint,
				 it->second->limits);
	    break;
	  case ::urdf::Joint::CONTINUOUS:
	    createContinuousJoint (jointName, position, urdfLinkInJoint);
	    break;
	  case ::urdf::Joint::PRISMATIC:
	    createTranslationJoint (jointName, position, urdfLinkInJoint,
				    it->second->limits);
	    break;
	  case ::urdf::Joint::FLOATING:
	    throw std::runtime_error ("FLOATING joints are not supported");
	    break;
	  case ::urdf::Joint::PLANAR:
	    throw std::runtime_error ("PLANAR joints are not supported");
	    break;
	  case ::urdf::Joint::FIXED:
	    createAnchorJoint (jointName, position);
	    break;
	  default:
	    std::ostringstream error;
	    error << "Unknown joint type: " << (int)it->second->type;
	    throw std::runtime_error (error.str ());
	  }
	}
      }

      void Parser::connectJoints (const JointPtr_t& rootJoint)
      {
	BOOST_FOREACH (const std::string& childName,
		       getChildrenJoint (rootJoint->name ())) {
	  MapHppJointType::const_iterator child = jointsMap_.find (childName);
	  if (child == jointsMap_.end () && !!child->second) {
	    throw std::runtime_error ("Failed to connect joint " + childName);
	  }
	  if (!child->second->parentJoint()) {
	    rootJoint->addChildJoint (child->second);
	  }
	  connectJoints (child->second);
	}
      }

      void Parser::addBodiesToJoints ()
      {
        for(MapHppJointType::const_iterator it = jointsMap_.begin();
	    it != jointsMap_.end(); ++it) {
	  // Retrieve associated URDF joint.
	  UrdfJointConstPtrType joint =
            model_.getJoint (removePrefix (it->first));
	  if (!joint && it->first != rootJointName_)
	    continue;

	  // Retrieve joint name.
	  std::string childLinkName;
	  UrdfLinkConstPtrType link;
	  if (it->first == rootJointName_) {
	    // Get root link
	    link = model_.getRoot ();
	    childLinkName = link->name;
	  }
	  else {
	    childLinkName = joint->child_link_name;
	    // Get child link.
	    link = model_.getLink (childLinkName);
	  }

	  if (!link) {
	    throw std::runtime_error (std::string ("Link ") + childLinkName +
				      std::string
				      (" not found, inconsistent model"));
	  }

	  // Retrieve inertial information.
	  boost::shared_ptr < ::urdf::Inertial> inertial = link->inertial;

	  fcl::Vec3f localCom (0., 0., 0.);
	  matrix3_t inertiaMatrix;
	  double mass = 0.;
	  if (inertial) {
	    localCom[0] = inertial->origin.position.x;
	    localCom[1] = inertial->origin.position.y;
	    localCom[2] = inertial->origin.position.z;

	    mass = inertial->mass;

	    inertiaMatrix (0, 0) = inertial->ixx;
	    inertiaMatrix (0, 1) = inertial->ixy;
	    inertiaMatrix (0, 2) = inertial->ixz;

	    inertiaMatrix (1, 0) = inertial->ixy;
	    inertiaMatrix (1, 1) = inertial->iyy;
	    inertiaMatrix (1, 2) = inertial->iyz;

	    inertiaMatrix (2, 0) = inertial->ixz;
	    inertiaMatrix (2, 1) = inertial->iyz;
	    inertiaMatrix (2, 2) = inertial->izz;

	    // Use joint normalization to properly reorient
	    // inertial frames.
	    if (it->first == rootJointName_) {}
	    else
	      if (link->parent_joint->type == ::urdf::Joint::REVOLUTE
		  || link->parent_joint->type == ::urdf::Joint::CONTINUOUS
		  || link->parent_joint->type == ::urdf::Joint::PRISMATIC) {
		MatrixHomogeneousType normalizedJointTransform
		  = normalizeFrameOrientation (link->parent_joint);

		MatrixHomogeneousType localComTransform;
		localComTransform.setIdentity ();
		localComTransform.setTranslation (localCom);
		MatrixHomogeneousType njtInverse =
		  normalizedJointTransform.inverse ();
		localComTransform = njtInverse * localComTransform;
		localCom = localComTransform.getTranslation ();

		fcl::Matrix3f R = normalizedJointTransform.getRotation ();
		fcl::Matrix3f RT = njtInverse.getRotation ();
		inertiaMatrix = RT * inertiaMatrix * R;
	      }
	  }
	  else {
	    hppDout (notice, "missing inertial information in link "
		     << childLinkName);
	  }

	  // Create dynamic body and fill inertial information.
	  Body* body = objectFactory_.createBody ();
	  assert (body);
	  body->name (prependPrefix (link->name));
	  hppDout (info, "creating Body with name " << body->name ()
		   << " at " << body);

	  body->mass (mass);
	  body->localCenterOfMass (localCom);
	  body->inertiaMatrix (inertiaMatrix);

	  // Link dynamic body to dynamic joint.
	  it->second->setLinkedBody (body);
	  it->second->linkName (prependPrefix (link->name));
	  hppDout (info,  "Linking body " << body->name () << " to joint "
		   << it->second->name ());

	  // Create geometric body and fill geometry information.
	  if (link->collision) {
	    JointPtr_t hppJoint = it->second;
	    addSolidComponentToJoint (link, hppJoint);
	  }
	}
      }

      Parser::MatrixHomogeneousType
      Parser::computeBodyAbsolutePosition
      (const Parser::UrdfLinkConstPtrType& link, const ::urdf::Pose& pose)
      {
	MatrixHomogeneousType linkPositionInParentJoint = poseToMatrix (pose);

	MatrixHomogeneousType parentJointInWorld;
	if (link == model_.getRoot ()) {
	  parentJointInWorld.setIdentity ();
	}
	else {
	  parentJointInWorld =
	    findJoint (prependPrefix (link->parent_joint->name))
              ->currentTransformation ();
	}
	// Denormalize orientation if this is an actuated joint.
	if (link->parent_joint
	    && (link->parent_joint->type == ::urdf::Joint::REVOLUTE
		|| link->parent_joint->type == ::urdf::Joint::CONTINUOUS
		|| link->parent_joint->type == ::urdf::Joint::PRISMATIC)) {
	  MatrixHomogeneousType inverse =
	    normalizeFrameOrientation (link->parent_joint).inverse ();
	  parentJointInWorld = parentJointInWorld * inverse;
	}

	MatrixHomogeneousType position = parentJointInWorld *
	  linkPositionInParentJoint;
	return position;
      }

      void Parser::buildMesh (const std::string& name,
                              const ::urdf::Vector3& scale,
			      const aiScene* scene,
			      const aiNode* node,
			      std::vector<unsigned>& subMeshIndexes,
			      const Parser::PolyhedronPtrType& mesh)
      {
	if (!node) return;

	aiMatrix4x4 transform = node->mTransformation;
	aiNode *pnode = node->mParent;
	while (pnode)
	  {
	    // Don't convert to y-up orientation, which is what the root node in
	    // Assimp does
	    if (pnode->mParent != NULL)
	      transform = pnode->mTransformation * transform;
	    pnode = pnode->mParent;
	  }

	for (uint32_t i = 0; i < node->mNumMeshes; i++) {
	  aiMesh* input_mesh = scene->mMeshes[node->mMeshes[i]];

	  unsigned oldNbPoints = mesh->num_vertices;
	  unsigned oldNbTriangles = mesh->num_tris;

	  // Add the vertices
	  for (uint32_t j = 0; j < input_mesh->mNumVertices; j++) {
	    aiVector3D p = input_mesh->mVertices[j];
	    p *= transform;
	    vertices_.push_back (fcl::Vec3f (p.x * scale.x,
					     p.y * scale.y,
					     p.z * scale.z));
	  }

	  // add the indices
	  for (uint32_t j = 0; j < input_mesh->mNumFaces; j++) {
	    aiFace& face = input_mesh->mFaces[j];
            if (face.mNumIndices != 3) {
              std::stringstream ss;
              ss << "Mesh " << name << " has a face with "
                << face.mNumIndices << " vertices. This is not supported\n";
              ss << "Node name is: " << node->mName.C_Str() << "\n";
              ss << "Mesh index: " << i << "\n";
              ss << "Face index: " << j << "\n";
              throw std::invalid_argument (ss.str());
            }
	    // FIXME: can add only triangular faces.
	    triangles_.push_back (fcl::Triangle
				  (oldNbPoints + face.mIndices[0],
				   oldNbPoints + face.mIndices[1],
				   oldNbPoints + face.mIndices[2]));
	  }

	  // Save submesh triangles indexes interval.
	  if (subMeshIndexes.size () == 0)
	    subMeshIndexes.push_back (0);

	  subMeshIndexes.push_back (oldNbTriangles + input_mesh->mNumFaces);
	}

	for (uint32_t i=0; i < node->mNumChildren; ++i) {
	  buildMesh(name, scale, scene, node->mChildren[i], subMeshIndexes, mesh);
	}
      }

      void Parser::meshFromAssimpScene (const std::string& name,
					const ::urdf::Vector3& scale,
					const aiScene* scene,
					const Parser::PolyhedronPtrType& mesh)
      {
	if (!scene->HasMeshes())
	  {
	    throw std::runtime_error (std::string ("No meshes found in file ")+
				      name);
	  }

	std::vector<unsigned> subMeshIndexes;
	int res = mesh->beginModel ();
	if (res != fcl::BVH_OK) {
	  std::ostringstream error;
	  error << "fcl BVHReturnCode = " << res;
	  throw std::runtime_error (error.str ());
	}
	vertices_.clear ();
	triangles_.clear ();
	buildMesh (name, scale, scene, scene->mRootNode, subMeshIndexes, mesh);
	mesh->addSubModel (vertices_, triangles_);
	mesh->endModel ();

      }

      void Parser::loadPolyhedronFromResource
      (const std::string& resource_path, const ::urdf::Vector3& scale,
       const PolyhedronPtrType& polyhedron)
      {
       Assimp::Importer importer;
        // set list of ignored parameters (parameters used for rendering)
       importer.SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS,
                        aiComponent_TANGENTS_AND_BITANGENTS|
                        aiComponent_COLORS |
                        aiComponent_BONEWEIGHTS |
                        aiComponent_ANIMATIONS |
                        aiComponent_LIGHTS |
                        aiComponent_CAMERAS|
                        aiComponent_TEXTURES |
                        aiComponent_TEXCOORDS |
                        aiComponent_MATERIALS |
                        aiComponent_NORMALS
                    );
       importer.SetIOHandler(new ResourceIOSystem());
       const aiScene* scene = importer.ReadFile(resource_path, aiProcess_SortByPType|aiProcess_Triangulate | aiProcess_RemoveComponent | aiProcess_JoinIdenticalVertices);
	if (!scene) {
	  throw std::runtime_error (std::string ("Could not load resource ") +
				    resource_path + std::string ("\n") +
				    importer.GetErrorString ());
	}

	meshFromAssimpScene (resource_path, scale, scene, polyhedron);
      }

      void Parser::addSolidComponentToJoint (const UrdfLinkConstPtrType& link,
					     const JointPtr_t& joint)
      {
	typedef std::vector < boost::shared_ptr < ::urdf::Collision > >
	  Collisions_t;
	Collisions_t collisions (link->collision_array);

	std::size_t objectId = 0;
	for (Collisions_t::iterator it = collisions.begin ();
	     it != collisions.end (); ++it) {

	  std::ostringstream oss;
	  oss << link->name << "_" << objectId;
	  std::string objectName (oss.str ());
	  ++objectId;
	boost::shared_ptr < ::urdf::Collision> collision = *it;
	fcl::CollisionGeometryPtr_t geometry;

	// Handle the case where collision geometry is a mesh
	if (collision->geometry->type == ::urdf::Geometry::MESH) {
	  boost::shared_ptr < ::urdf::Mesh> collisionGeometry
	    = boost::dynamic_pointer_cast< ::urdf::Mesh> (collision->geometry);
	  std::string collisionFilename = collisionGeometry->filename;
	  ::urdf::Vector3 scale = collisionGeometry->scale;
	  // Create FCL mesh by parsing Collada file.
	  PolyhedronPtrType  polyhedron (new PolyhedronType);

	  // name is stored in link->name
	  loadPolyhedronFromResource (collisionFilename, scale, polyhedron);
	  geometry = polyhedron;
	}
	// Handle the case where collision geometry is a cylinder
	// Use FCL capsules for cylinders
	else if (collision->geometry->type == ::urdf::Geometry::CYLINDER) {
		  boost::shared_ptr < ::urdf::Cylinder> collisionGeometry
		    = boost::dynamic_pointer_cast< ::urdf::Cylinder>
		    (collision->geometry);
	
		  double radius = collisionGeometry->radius;
		  double length = collisionGeometry->length;
	
		  // Create fcl capsule geometry.
		  geometry = fcl::CollisionGeometryPtr_t
		    (new fcl::Capsule (radius, length));
	}
	// Handle the case where collision geometry is a box.
	else if (collision->geometry->type == ::urdf::Geometry::BOX) {
		  boost::shared_ptr < ::urdf::Box> collisionGeometry
		    = boost::dynamic_pointer_cast< ::urdf::Box> (collision->geometry);
	
		  double x = collisionGeometry->dim.x;
		  double y = collisionGeometry->dim.y;
		  double z = collisionGeometry->dim.z;
	
		  geometry = fcl::CollisionGeometryPtr_t (new fcl::Box (x, y, z));
	}
 	// Handle the case where collision geometry is a sphere.
	else if (collision->geometry->type == ::urdf::Geometry::SPHERE) {
   	  boost::shared_ptr < ::urdf::Sphere> collisionGeometry
   	    = boost::dynamic_pointer_cast< ::urdf::Sphere> (collision->geometry);

      double radius = collisionGeometry->radius;

      geometry = fcl::CollisionGeometryPtr_t (new fcl::Sphere (radius));
  }
	else throw std::runtime_error (std::string ("Unknown geometry type :"));// +
    //collision->geometry->type);

	// Compute body position in world frame.
	MatrixHomogeneousType position =
	  computeBodyAbsolutePosition (link, collision->origin);
	if (geometry) {
	  CollisionObjectPtr_t collisionObject (CollisionObject::create
              (geometry, position, prependPrefix (objectName)));

	  // Add solid component.
	  Body* body = joint->linkedBody ();
	  assert (body);
	  body->addInnerObject (collisionObject, true, true);
	  hppDout (info, "Adding object " << collisionObject->name ()
		   << " to body " << body->name ());
	}
	}
      }

      void Parser::fillGaze ()
      {
	HumanoidRobotPtr_t robot = HPP_DYNAMIC_PTR_CAST (HumanoidRobot,
							 robot_);
	if (!robot) {
	  throw std::runtime_error ("Robot is not a humanoid");
	}
	MapHppJointType::const_iterator gaze =
	  jointsMap_.find (gazeJointName_);
	JointPtr_t gazeJoint = gaze->second;
	robot->gazeJoint (gazeJoint);
	vector3_t dir, origin;
	// Gaze direction is defined by the gaze joint local
	// orientation.
	dir[0] = 1;
	dir[1] = 0;
	dir[2] = 0;
	// Gaze position should is defined by the gaze joint local
	// origin.
	origin[0] = 0;
	origin[1] = 0;
	origin[2] = 0;
	robot->gaze (dir, origin);
      }

      std::vector<std::string>
      Parser::getChildrenJoint (const std::string& jointName)
      {
	std::vector<std::string> result;
	getChildrenJoint (jointName, result);
	return result;
      }

      void
      Parser::getChildrenJoint (const std::string& jointName,
				std::vector<std::string>& result)
      {
	typedef boost::shared_ptr < ::urdf::Joint> jointPtr_t;

	boost::shared_ptr <const ::urdf::Joint> joint =
	  model_.getJoint (removePrefix (jointName));

	if (!joint
            && removePrefix (jointName).compare (0, 10, "base_joint") != 0) {
	  std::string msg ("Failed to retrieve children joints of joint ");
	  msg += std::string (jointName);
	  throw std::runtime_error (msg);
	}

	boost::shared_ptr <const ::urdf::Link> childLink;
	if (removePrefix (jointName).compare (0, 10, "base_joint") == 0)
	  childLink = model_.getRoot ();
	else
	  childLink = model_.getLink (joint->child_link_name);

	if (!childLink)
	  {
	    std::string msg ("Failed to retrieve children link of joint ");
	    msg += std::string (jointName);
	    throw std::runtime_error (msg);
	  }

	const std::vector<jointPtr_t>& jointChildren =
	  childLink->child_joints;

	BOOST_FOREACH (const jointPtr_t& joint, jointChildren)
	  {
	    if (jointsMap_.count(prependPrefix (joint->name)) > 0)
	      result.push_back (prependPrefix (joint->name));
	    else
	      getChildrenJoint (prependPrefix (joint->name), result);
	  }
      }

      void
      Parser::createFreeflyerJoint (const std::string& name,
				    const MatrixHomogeneousType& mat,
				    DevicePtr_t robot)
      {
	JointPtr_t joint, parent;
	std::string jointName = name + "_xyz";
	if (jointsMap_.find (jointName) != jointsMap_.end ()) {
	  throw std::runtime_error (std::string ("Duplicated joint ") +
				    jointName);
	}
	// Translation along xyz
	joint = objectFactory_.createJointTranslation3 (mat);
	joint->name (jointName);
	jointsMap_[jointName] = joint;
	joint->lowerBound (0, -numeric_limits<double>::infinity());
	joint->upperBound (0, +numeric_limits<double>::infinity());
	joint->lowerBound (1, -numeric_limits<double>::infinity());
	joint->upperBound (1, +numeric_limits<double>::infinity());
	joint->lowerBound (2, -numeric_limits<double>::infinity());
	joint->upperBound (2, +numeric_limits<double>::infinity());
        if (baseJoint_) baseJoint_->addChildJoint (joint);
        else if (robot) robot->rootJoint (joint);
	parent = joint;
	// joint SO3
	joint = objectFactory_.createJointSO3 (mat);
	jointName = name + "_SO3";
	if (jointsMap_.find (jointName) != jointsMap_.end ()) {
	  throw std::runtime_error (std::string ("Duplicated joint ") +
				    jointName);
	}
	joint->name (jointName);
	jointsMap_[jointName] = joint;
	parent->addChildJoint (joint);
      }

      void
      Parser::createPlanarJoint (const std::string& name,
				 const MatrixHomogeneousType& mat,
				 DevicePtr_t robot)
      {
	JointPtr_t joint, parent;
	const fcl::Vec3f T = mat.getTranslation ();
	std::string jointName = name + "_xy";
	if (jointsMap_.find (jointName) != jointsMap_.end ()) {
	  throw std::runtime_error (std::string ("Duplicated joint ") +
				    jointName);
	}
	// Translation along x
	fcl::Matrix3f permutation;
	joint = objectFactory_.createJointTranslation2 (mat);
	joint->name (jointName);
	jointsMap_[jointName] = joint;
	joint->lowerBound (0, -numeric_limits<double>::infinity());
	joint->upperBound (0, +numeric_limits<double>::infinity());
	joint->lowerBound (1, -numeric_limits<double>::infinity());
	joint->upperBound (1, +numeric_limits<double>::infinity());
        if (baseJoint_) baseJoint_->addChildJoint (joint);
        else if (robot) robot->rootJoint (joint);
	parent = joint;
	// Rotation along z
	permutation (0,0) = 0; permutation (0,1) = 0; permutation (0,2) = -1;
	permutation (1,0) = 0; permutation (1,1) = 1; permutation (1,2) =  0;
	permutation (2,0) = 1; permutation (2,1) = 0; permutation (2,2) =  0;
	fcl::Transform3f pos;
	pos.setRotation (permutation * mat.getRotation ());
	pos.setTranslation (T);
	joint = objectFactory_.createUnBoundedJointRotation (pos);
	jointName = name + "_rz";
	if (jointsMap_.find (jointName) != jointsMap_.end ()) {
	  throw std::runtime_error (std::string ("Duplicated joint ") +
				    jointName);
	}
	joint->name (jointName);
	pos.inverse ();
	joint->linkInJointFrame (pos);
	jointsMap_[jointName] = joint;
	joint->lowerBound (0, -numeric_limits<double>::infinity());
	joint->upperBound (0, +numeric_limits<double>::infinity());
	parent->addChildJoint (joint);
	parent = joint;
      }

      JointPtr_t
      Parser::createRotationJoint (const std::string& name,
				   const MatrixHomogeneousType& mat,
				   const MatrixHomogeneousType& urdfLinkInJoint,
				   const Parser::UrdfJointLimitsPtrType& limits)
      {
	JointPtr_t joint;
	if (jointsMap_.find (name) != jointsMap_.end ()) {
	  throw std::runtime_error (std::string ("Duplicated joint ") +
				    name);
	}

	joint = objectFactory_.createBoundedJointRotation (mat);
	joint->name (name);
	joint->linkInJointFrame (urdfLinkInJoint);
	if (limits) {
	  joint->isBounded (0, true);
	  joint->lowerBound (0, limits->lower);
	  joint->upperBound (0, limits->upper);
	}
	jointsMap_[name] = joint;
	return joint;
      }

      JointPtr_t Parser::createContinuousJoint
      (const std::string& name, const MatrixHomogeneousType& mat,
       const MatrixHomogeneousType& urdfLinkInJoint)
      {
	JointPtr_t joint;
	if (jointsMap_.find (name) != jointsMap_.end ()) {
	  throw std::runtime_error (std::string ("Duplicated joint ") +
				    name);
	}

	joint = objectFactory_.createUnBoundedJointRotation (mat);
	joint->name (name);
	joint->linkInJointFrame (urdfLinkInJoint);
	jointsMap_[name] = joint;
	return joint;
      }

      JointPtr_t Parser::createTranslationJoint
      (const std::string& name, const MatrixHomogeneousType& mat,
       const MatrixHomogeneousType& urdfLinkInJoint,
       const Parser::UrdfJointLimitsPtrType& limits)
      {
	JointPtr_t joint;
	if (jointsMap_.find (name) != jointsMap_.end ()) {
	  throw std::runtime_error (std::string ("Duplicated joint ") +
				    name);
	}

	joint = objectFactory_.createJointTranslation (mat);
	joint->name (name);
	joint->linkInJointFrame (urdfLinkInJoint);
	if (limits) {
	  joint->isBounded (0, true);
	  joint->lowerBound (0, limits->lower);
	  joint->upperBound (0, limits->upper);
	} else {
	  joint->isBounded (0, false);
	  joint->lowerBound
	    (0, -numeric_limits <double>::infinity ());
	  joint->upperBound
	    (0, numeric_limits <double>::infinity ());
	}
	jointsMap_[name] = joint;
	return joint;
      }

      JointPtr_t
      Parser::createAnchorJoint (const std::string& name,
				 const MatrixHomogeneousType& mat)
      {
	JointPtr_t joint;
	if (jointsMap_.find (name) != jointsMap_.end ()) {
	  throw std::runtime_error (std::string ("Duplicated joint ") +
				    name);
	}

	joint = objectFactory_.createJointAnchor (mat);
	joint->name (name);
	jointsMap_[name] = joint;
	return joint;
      }

      JointPtr_t
      Parser::findJoint (const std::string& jointName)
      {
	Parser::MapHppJointType::const_iterator it =
	  jointsMap_.find (jointName);
	if (it == jointsMap_.end ()) {
	  throw std::runtime_error ("Joint " + jointName + " not found.");
	}
	return it->second;
      }

      Parser::MatrixHomogeneousType
      Parser::poseToMatrix (::urdf::Pose p)
      {
	// Fill rotation part: convert quaternion to rotation matrix.
	fcl::Quaternion3f quat (p.rotation.w, p.rotation.x, p.rotation.y,
				p.rotation.z);
	// Fill translation part.
	fcl::Vec3f T;
	T [0] = p.position.x;
	T [1] = p.position.y;
	T [2] = p.position.z;

	return MatrixHomogeneousType (quat, T);
      }

      Parser::MatrixHomogeneousType
      Parser::getPoseInReferenceFrame (const std::string& referenceJointName,
				       const std::string& currentJointName)
      {
	if (referenceJointName == currentJointName)
	  return poseToMatrix
	    (model_.getJoint
	     (currentJointName)->parent_to_joint_origin_transform);

	// Retrieve corresponding joint in URDF tree.
	UrdfJointConstPtrType joint = model_.getJoint (currentJointName);
	if (!joint)
	  {
	    hppDout (error,
		     "Failed to retrieve parent while computing joint position");
	    MatrixHomogeneousType result;
	    result.setIdentity ();
	    return result;
	  }

	// Get transform from parent link to joint.
	::urdf::Pose jointToParentTransform =
	    joint->parent_to_joint_origin_transform;

	MatrixHomogeneousType transform = poseToMatrix (jointToParentTransform);

	// Get parent joint name.
	std::string parentLinkName = joint->parent_link_name;
	UrdfLinkConstPtrType parentLink = model_.getLink (parentLinkName);

	if (!parentLink)
	  return transform;
	UrdfJointConstPtrType parentJoint = parentLink->parent_joint;
	if (!parentJoint)
	  return transform;

	// Compute previous transformation with current one.
	transform =
	  getPoseInReferenceFrame (referenceJointName,
				   parentJoint->name) * transform;
	return transform;
      }

      void Parser::parseFromParameter (const std::string& parameterName)
      {
	// Reset the attributes to avoid problems when loading
	// multiple robots using the same object.
	model_.clear ();
	rootJoint_ = 0;
	jointsMap_.clear ();

	// Parse urdf model.
	if (!model_.initParam (parameterName)) {
	  throw std::runtime_error ("Failed to read parameter " +
				    parameterName);
	}
	buildRobot ();
      }

      void Parser::parse (const std::string& filename)
      {
	hppDout (info, "filename: " << filename);
	resource_retriever::Retriever resourceRetriever;

	resource_retriever::MemoryResource resource =
	  resourceRetriever.get(filename);
	std::string robotDescription;
	robotDescription.resize(resource.size);
	unsigned i = 0;
	for (; i < resource.size; ++i)
	  robotDescription[i] = resource.data.get()[i];

	// Reset the attributes to avoid problems when loading
	// multiple robots using the same object.
	model_.clear ();
	rootJoint_ = 0;
	jointsMap_.clear ();

	// Parse urdf model.
	if (!model_.initString (robotDescription)) {
	  throw std::runtime_error ("Failed to open urdf file. "
				    "robotDescription:\n" + robotDescription);
	}
	buildRobot ();
      }

      void Parser::buildRobot ()
      {
	// Get names of special joints.
	findSpecialJoints ();

	// Look for joints in the URDF model tree.
	parseJoints ();

	// Create the kinematic tree.
	// We iterate over the URDF root joints to connect them to the
	// root link that we added "manually" before. Then we iterate
	// in the whole tree using the connectJoints method.
	boost::shared_ptr <const ::urdf::Link> rootLink = model_.getRoot ();
	if (!rootLink)
	  {
	    hppDout (error, "URDF model is missing a root link");
	    throw std::runtime_error ("URDF model is missing a root link");
	  }

	connectJoints (rootJoint_);
	// Add corresponding body (link) to each joint.
	addBodiesToJoints ();
      }
    } // end of namespace urdf.
  } // end of namespace model.
} // end of namespace  hpp.
