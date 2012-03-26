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
 * \file src/polyhedron-loader.cc
 *
 * \brief Implementation of polyhedron loader functions.
 */

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <assimp/assimp.hpp>
#include <assimp/aiScene.h>
#include <assimp/aiPostProcess.h>
#include <assimp/IOStream.h>
#include <assimp/IOSystem.h>

#include <resource_retriever/retriever.h>

#include "hpp/model/urdf/polyhedron-loader.hh"

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

	size_t Write (const void* buffer, size_t size, size_t count) { return 0; }

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
	Assimp::IOStream* Open(const char* file, const char* mode)
	{
	  assert (mode == std::string("r") || mode == std::string("rb"));

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
      void buildMesh (const aiScene* scene,
		      const aiNode* node,
		      const CkppPolyhedronShPtr& mesh)
      {
	if (!node)
	  {
	    return;
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

	    // mesh->reserveNPoints (input_mesh->mNumVertices);

	    // Add the vertices
	    for (uint32_t j = 0; j < input_mesh->mNumVertices; j++)
	      {
		aiVector3D p = input_mesh->mVertices[j];
		p *= transform;
		mesh->addPoint (CkitPoint3 (p.x, p.y, p.z));
	      }

	    // add the indices
	    for (uint32_t j = 0; j < input_mesh->mNumFaces; j++)
	      {
		aiFace& face = input_mesh->mFaces[j];
		// FIXME: can add only triangular faces.
		mesh->addTriangle (face.mIndices[0],
				   face.mIndices[1],
				   face.mIndices[2]);
	      }
	  }

	for (uint32_t i=0; i < node->mNumChildren; ++i)
	  {
	    buildMesh(scene, node->mChildren[i], mesh);
	  }
      }

      void loadTexture(const std::string& resource_path)
      {
      }

      // Mostly cribbed from gazebo
      void loadMaterialsForMesh (const std::string& resource_path,
				 const aiScene* scene,
				 const CkppPolyhedronShPtr& polyhedron)
      {
	// std::vector<Ogre::MaterialPtr> material_lookup;

	// for (uint32_t i = 0; i < scene->mNumMaterials; i++)
	//   {
	//     std::stringstream ss;
	//     ss << resource_path << "Material" << i;
	//     Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().create(ss.str(), ROS_PACKAGE_NAME, true);
	//     material_lookup.push_back(mat);

	//     Ogre::Technique* tech = mat->getTechnique(0);
	//     Ogre::Pass* pass = tech->getPass(0);

	//     aiMaterial *amat = scene->mMaterials[i];

	//     Ogre::ColourValue diffuse(1.0, 1.0, 1.0, 1.0);
	//     Ogre::ColourValue specular(1.0, 1.0, 1.0, 1.0);
	//     Ogre::ColourValue ambient(0.5, 0.5, 0.5, 1.0);

	//     for (uint32_t j=0; j < amat->mNumProperties; j++)
	//       {
	// 	aiMaterialProperty *prop = amat->mProperties[j];
	// 	std::string propKey = prop->mKey.data;

	// 	if (propKey == "$tex.file")
	// 	  {
	// 	    aiString texName;
	// 	    aiTextureMapping mapping;
	// 	    uint32_t uvIndex;
	// 	    amat->GetTexture(aiTextureType_DIFFUSE,0, &texName, &mapping, &uvIndex);

	// 	    // Assume textures are in paths relative to the mesh
	// 	    std::string texture_path = fs::path(resource_path).parent_path().string() + "/" + texName.data;
	// 	    loadTexture(texture_path);
	// 	    Ogre::TextureUnitState* tu = pass->createTextureUnitState();
	// 	    tu->setTextureName(texture_path);
	// 	  }
	// 	else if (propKey == "$clr.diffuse")
	// 	  {
	// 	    aiColor3D clr;
	// 	    amat->Get(AI_MATKEY_COLOR_DIFFUSE, clr);
	// 	    diffuse = Ogre::ColourValue(clr.r, clr.g, clr.b);
	// 	  }
	// 	else if (propKey == "$clr.ambient")
	// 	  {
	// 	    aiColor3D clr;
	// 	    amat->Get(AI_MATKEY_COLOR_AMBIENT, clr);

	// 	    // Most of are DAE files don't have ambient color defined
	// 	    if (clr.r > 0 && clr.g > 0 && clr.b > 0)
	// 	      {
	// 		ambient = Ogre::ColourValue(clr.r, clr.g, clr.b);
	// 	      }
	// 	  }
	// 	else if (propKey == "$clr.specular")
	// 	  {
	// 	    aiColor3D clr;
	// 	    amat->Get(AI_MATKEY_COLOR_SPECULAR, clr);
	// 	    specular = Ogre::ColourValue(clr.r, clr.g, clr.b);
	// 	  }
	// 	else if (propKey == "$clr.emissive")
	// 	  {
	// 	    aiColor3D clr;
	// 	    amat->Get(AI_MATKEY_COLOR_EMISSIVE, clr);
	// 	    mat->setSelfIllumination(clr.r, clr.g, clr.b);
	// 	  }
	// 	else if (propKey == "$clr.opacity")
	// 	  {
	// 	    float o;
	// 	    amat->Get(AI_MATKEY_OPACITY, o);
	// 	    diffuse.a = o;
	// 	  }
	// 	else if (propKey == "$mat.shininess")
	// 	  {
	// 	    float s;
	// 	    amat->Get(AI_MATKEY_SHININESS, s);
	// 	    mat->setShininess(s);
	// 	  }
	// 	else if (propKey == "$mat.shadingm")
	// 	  {
	// 	    int model;
	// 	    amat->Get(AI_MATKEY_SHADING_MODEL, model);
	// 	    switch(model)
	// 	      {
	// 	      case aiShadingMode_Flat:
	// 		mat->setShadingMode(Ogre::SO_FLAT);
	// 		break;
	// 	      case aiShadingMode_Phong:
	// 		mat->setShadingMode(Ogre::SO_PHONG);
	// 		break;
	// 	      case aiShadingMode_Gouraud:
	// 	      default:
	// 		mat->setShadingMode(Ogre::SO_GOURAUD);
	// 		break;
	// 	      }
	// 	  }
	//       }

	//     int mode = aiBlendMode_Default;
	//     amat->Get(AI_MATKEY_BLEND_FUNC, mode);
	//     switch(mode)
	//       {
	//       case aiBlendMode_Additive:
	// 	mat->setSceneBlending(Ogre::SBT_ADD);
	// 	break;
	//       case aiBlendMode_Default:
	//       default:
	// 	{
	// 	  if (diffuse.a < 0.99)
	// 	    {
	// 	      pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
	// 	    }
	// 	  else
	// 	    {
	// 	      pass->setSceneBlending(Ogre::SBT_REPLACE);
	// 	    }
	// 	}
	// 	break;
	//       }

	//     mat->setAmbient(ambient);
	//     mat->setDiffuse(diffuse);
	//     specular.a = diffuse.a;
	//     mat->setSpecular(specular);
	//   }

	// for (uint32_t i = 0; i < mesh->getNumSubMeshes(); ++i)
	//   {
	//     mesh->getSubMesh(i)->setMaterialName(material_lookup[scene->mMeshes[i]->mMaterialIndex]->getName());
	//   }
      }

      void
      meshFromAssimpScene (const std::string& name,
			   const aiScene* scene,
			   const CkppPolyhedronShPtr& mesh)
      {
	if (!scene->HasMeshes())
	  {
	    boost::format fmt
	      ("No meshes found in file [%s]");
	    fmt % name;
	    throw std::runtime_error (fmt.str ());
	  }

	buildMesh(scene, scene->mRootNode, mesh);

	loadMaterialsForMesh(name, scene, mesh);
      }

      void
      loadPolyhedronFromResource(const std::string& resource_path,
				 const CkppPolyhedronShPtr& polyhedron)
      {
	Assimp::Importer importer;
	importer.SetIOHandler(new ResourceIOSystem());
	const aiScene* scene = importer.ReadFile(resource_path, aiProcess_SortByPType|aiProcess_GenNormals|aiProcess_Triangulate|aiProcess_GenUVCoords|aiProcess_FlipUVs);
	if (!scene)
	  {
	    boost::format fmt
	      ("Could not load resource [%s]: %s");
	    fmt % resource_path % importer.GetErrorString();
	    throw std::runtime_error (fmt.str ());
	  }

	meshFromAssimpScene (resource_path, scene, polyhedron);
      }
      
    } // end of namespace urdf.
  } // end of namespace model.
} // end of namespace  hpp.