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

/// \file src/util.cc
///
/// \brief Implementation of utility functions.

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <assimp/assimp.hpp>
#include <assimp/aiScene.h>
#include <assimp/aiPostProcess.h>
#include <assimp/IOStream.h>
#include <assimp/IOSystem.h>

#include <resource_retriever/retriever.h>

#include <hpp/util/debug.hh>
#include <hpp/util/assertion.hh>

#include <hpp/model/types.hh>

#include "hpp/model/urdf/util.hh"

namespace fs = boost::filesystem;

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

      // Mostly stolen from gazebo
      bool buildMesh (const ::urdf::Vector3& scale,
		      const aiScene* scene,
		      const aiNode* node,
		      std::vector<unsigned>& subMeshIndexes,
		      const CkppPolyhedronShPtr& mesh)
      {
	if (!node)
	  {
	    return true;
	  }

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

	aiMatrix3x3 rotation(transform);
	aiMatrix3x3 inverse_transpose_rotation(rotation);
	inverse_transpose_rotation.Inverse();
	inverse_transpose_rotation.Transpose();

	for (uint32_t i = 0; i < node->mNumMeshes; i++)
	  {
	    aiMesh* input_mesh = scene->mMeshes[node->mMeshes[i]];

	    unsigned oldNbPoints = mesh->countPoints ();
	    unsigned oldNbTriangles = mesh->countTriangles ();

	    // Add the vertices
	    for (uint32_t j = 0; j < input_mesh->mNumVertices; j++)
	      {
		aiVector3D p = input_mesh->mVertices[j];
		p *= transform;
		mesh->addPoint (CkitPoint3 (p.x * scale.x,
					    p.y * scale.y,
					    p.z * scale.z));
	      }

	    // add the indices
	    for (uint32_t j = 0; j < input_mesh->mNumFaces; j++)
	      {
		aiFace& face = input_mesh->mFaces[j];
		// FIXME: can add only triangular faces.
		mesh->addTriangle (oldNbPoints + face.mIndices[0],
				   oldNbPoints + face.mIndices[1],
				   oldNbPoints + face.mIndices[2]);
	      }

	    // Save submesh triangles indexes interval.
	    if (subMeshIndexes.size () == 0)
	      subMeshIndexes.push_back (0);

	    subMeshIndexes.push_back (oldNbTriangles + input_mesh->mNumFaces);
	  }

	for (uint32_t i=0; i < node->mNumChildren; ++i)
	  {
	    buildMesh(scale, scene, node->mChildren[i], subMeshIndexes, mesh);
	  }

	return true;
      }

      void loadTexture(const std::string&)
      {
      }

      // Mostly cribbed from gazebo
      bool loadMaterialsForMesh (const std::string& resource_path,
				 const aiScene* scene,
				 const std::vector<unsigned> subMeshIndexes,
				 const CkppPolyhedronShPtr& mesh)
      {
	std::vector<CkppMaterial> materials;

	// Get all materials from scene and store them in a vector.
	for (uint32_t i = 0; i < scene->mNumMaterials; i++)
	  {
	    aiMaterial *amat = scene->mMaterials[i];

	    CkppMaterial material;
	    CkppColor diffuse (CkppColor (1.0, 1.0, 1.0, 1.0));
	    CkppColor specular (CkppColor (1.0, 1.0, 1.0, 1.0));
	    CkppColor ambient (CkppColor (0.5, 0.5, 0.5, 1.0));

	    for (uint32_t j=0; j < amat->mNumProperties; j++)
	      {
		aiMaterialProperty *prop = amat->mProperties[j];
		std::string propKey = prop->mKey.data;

		if (propKey == "$tex.file")
		  {
		    // FIXME: not supported
		    aiString texName;
		    aiTextureMapping mapping;
		    uint32_t uvIndex;
		    amat->GetTexture(aiTextureType_DIFFUSE,0, &texName, &mapping, &uvIndex);

		    // Assume textures are in paths relative to the mesh
		    std::string texture_path = fs::path(resource_path).parent_path().string() + "/" + texName.data;
		    loadTexture(texture_path);
		  }
		else if (propKey == "$clr.diffuse")
		  {
		    aiColor3D clr;
		    amat->Get(AI_MATKEY_COLOR_DIFFUSE, clr);
		    diffuse.red (clr.r);
		    diffuse.green (clr.g);
		    diffuse.blue (clr.b);
		  }
		else if (propKey == "$clr.ambient")
		  {
		    aiColor3D clr;
		    amat->Get(AI_MATKEY_COLOR_AMBIENT, clr);

		    // Most of are DAE files don't have ambient color defined
		    if (clr.r > 0 && clr.g > 0 && clr.b > 0)
		      {
			ambient.red (clr.r);
			ambient.green (clr.g);
			ambient.blue (clr.b);
		      }
		  }
		else if (propKey == "$clr.specular")
		  {
		    aiColor3D clr;
		    amat->Get(AI_MATKEY_COLOR_SPECULAR, clr);
		    specular.red (clr.r);
		    specular.green (clr.g);
		    specular.blue (clr.b);
		  }
		else if (propKey == "$clr.emissive")
		  {
		    // FIXME: not supported
		    aiColor3D clr;
		    amat->Get(AI_MATKEY_COLOR_EMISSIVE, clr);
		  }
		else if (propKey == "$clr.opacity")
		  {
		    float o;
		    amat->Get(AI_MATKEY_OPACITY, o);
		    diffuse.alpha (o);
		    specular.alpha (o);
		    ambient.alpha (o);
		  }
		else if (propKey == "$mat.shininess")
		  {
		    float s;
		    amat->Get(AI_MATKEY_SHININESS, s);
		    material.shininess (s);
		  }
		else if (propKey == "$mat.shadingm")
		  {
		    // FIXME: not supported
		    int model;
		    amat->Get(AI_MATKEY_SHADING_MODEL, model);
		  }
	      }

	    material.diffuseColor (diffuse);
	    material.specularColor (specular);
	    material.ambientColor (ambient);

	    materials.push_back (material);
	  }

	assert (subMeshIndexes.size () - 1 == materials.size ()
		&& "SubMeshIndexes and materials size mismatch");
	for (uint32_t i = 0; i < subMeshIndexes.size () - 1; ++i)
	  {
	    mesh->setMaterial (subMeshIndexes[i], subMeshIndexes[i + 1] - 1,
			       materials[scene->mMeshes[i]->mMaterialIndex]);
	  }

	return true;
      }

      bool
      meshFromAssimpScene (const std::string& name,
			   const ::urdf::Vector3& scale,
			   const aiScene* scene,
			   const CkppPolyhedronShPtr& mesh)
      {
	if (!scene->HasMeshes())
	  {
	    hppDout (error, "No meshes found in file " << name);
	    return false;
	  }

	std::vector<unsigned> subMeshIndexes;
	if (!buildMesh(scale, scene, scene->mRootNode, subMeshIndexes, mesh))
	  {
	    hppDout (error, "Could not build mesh.");
	    return false;
	  }

	if (!loadMaterialsForMesh(name, scene, subMeshIndexes, mesh))
	  {
	    hppDout (error, "Could not load materials for mesh.");
	    return false;
	  }
	
	return true;
      }

      bool
      loadPolyhedronFromResource(const std::string& resource_path,
				 const ::urdf::Vector3& scale,
				 const CkppPolyhedronShPtr& polyhedron)
      {
	Assimp::Importer importer;
	importer.SetIOHandler(new ResourceIOSystem());
	const aiScene* scene = importer.ReadFile(resource_path, aiProcess_SortByPType|aiProcess_GenNormals|aiProcess_Triangulate|aiProcess_GenUVCoords|aiProcess_FlipUVs);
	if (!scene)
	  {
	    hppDout (error, "Could not load resource " << resource_path);
	    hppDout (error, importer.GetErrorString ());
	    return false;
	  }

	if (!meshFromAssimpScene (resource_path, scale, scene, polyhedron))
	  {
	    hppDout (error, "Could not load mesh from assimp scene.");
	    return false;
	  }

	return true;
      }

      bool
      loadRobotModel (model::HumanoidRobotShPtr& device,
		      const std::string& modelName,
		      const std::string& urdfSuffix,
		      const std::string& srdfSuffix,
		      const std::string& rcpdfSuffix)
      {
	hpp::model::urdf::Parser urdfParser;
	hpp::model::srdf::Parser srdfParser;
	hpp::model::rcpdf::Parser rcpdfParser;

	std::string urdfPath = "package://" + modelName + "_description/urdf/"
	  + modelName + urdfSuffix + ".urdf";
	std::string srdfPath = "package://" + modelName + "_description/srdf/"
	  + modelName + srdfSuffix + ".srdf";
	std::string rcpdfPath = "package://" + modelName + "_description/rcpdf/"
	  + modelName + rcpdfSuffix + ".rcpdf";

	// Build robot model from URDF.
	device = urdfParser.parse (urdfPath);
	if (!device)
	  {
	    hppDout (error, "Could not parse URDF file.");
	    return false;
	  }
	hppDout (notice, "Finished parsing URDF file.");

	device->isVisible (false);

	// Set Collision Check Pairs
	if (!srdfParser.parse (urdfPath, srdfPath, device))
	  {
	    hppDout (error, "Could not parse SRDF file.");
	  }
	else {
	  hppDout (notice, "Finished parsing SRDF file.");
	}

	// Set robot in a half-sitting configuration;
	hpp::model::srdf::Parser::HppConfigurationType halfSittingConfig
	  = srdfParser.getHppReferenceConfig ("all", "half_sitting");

	device->hppSetCurrentConfig (halfSittingConfig);

	// Set contact point properties.
	if (!rcpdfParser.parse (rcpdfPath, device))
	  {
	    hppDout (error, "Could not parse RCPDF file.");
	  }
	else {
	  hppDout (notice, "Finished parsing RCPDF file.");
	}

	return true;
      }

    } // end of namespace urdf.
  } // end of namespace model.
} // end of namespace  hpp.
