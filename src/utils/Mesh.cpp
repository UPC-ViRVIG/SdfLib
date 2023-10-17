#include "SdfLib/utils/Mesh.h"
#include <iostream>
#include <assert.h>
#include <spdlog/spdlog.h>

namespace sdflib
{
#ifdef SDFLIB_ASSIMP_AVAILABLE
Mesh::Mesh(std::string filePath)
{
    Assimp::Importer import;
    const aiScene *scene = import.ReadFile(filePath, aiProcess_Triangulate | aiProcess_FlipUVs);
    
    if(!scene || !scene->mRootNode || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE)
    {
        SPDLOG_ERROR("Error with Assimp: {}", import.GetErrorString());
        return;
    }

    if(!scene->HasMeshes())
    {
        SPDLOG_ERROR("The model {} does not have any mesh assosiated", filePath);
    }

    initMesh(scene->mMeshes[0]);       
}

Mesh::Mesh(const aiMesh* mesh)
{
    initMesh(mesh);
}
#endif

Mesh::Mesh(glm::vec3* vertices, uint32_t numVertices,
         uint32_t* indices, uint32_t numIndices)
{
    mVertices.resize(numVertices);
    std::memcpy(mVertices.data(), vertices, sizeof(glm::vec3) * numVertices);

    mIndices.resize(numIndices);
    std::memcpy(mIndices.data(), indices, sizeof(uint32_t) * numIndices);
}

#ifdef SDFLIB_ASSIMP_AVAILABLE
void Mesh::initMesh(const aiMesh* mesh)
{
    if(!(mesh->mPrimitiveTypes & aiPrimitiveType_TRIANGLE))
    {
        SPDLOG_ERROR("The model must be a triangle mesh");
    }

    // Copy vertices
    SPDLOG_INFO("Model num vertices: {}", mesh->mNumVertices);
    mVertices.resize(mesh->mNumVertices);
    const size_t vertexSize = sizeof(decltype(mVertices[0]));
    assert(vertexSize == sizeof(decltype(mesh->mVertices[0])));
    std::memcpy(mVertices.data(), mesh->mVertices, mesh->mNumVertices * vertexSize);

    // Calculate bounding box
    computeBoundingBox();
    SPDLOG_INFO("BB min: {}, {}, {}", mBBox.min.x, mBBox.min.y, mBBox.min.z);
    SPDLOG_INFO("BB max: {}, {}, {}", mBBox.max.x, mBBox.max.y, mBBox.max.z);

    // Copy indices
    SPDLOG_INFO("Model num faces: {}", mesh->mNumFaces);
    mIndices.resize(3 * mesh->mNumFaces);

    for(size_t f=0, i=0; f < mesh->mNumFaces; f++, i += 3)
    {
        mIndices[i] = mesh->mFaces[f].mIndices[0];
        mIndices[i + 1] = mesh->mFaces[f].mIndices[1];
        mIndices[i + 2] = mesh->mFaces[f].mIndices[2];
    }

    // Copy normals
	if (mesh->HasNormals())
	{
		mNormals.resize(mesh->mNumVertices);
		const size_t normalSize = sizeof(decltype(mNormals[0]));
		assert(normalSize == sizeof(decltype(mesh->mNormals[0])));
		std::memcpy(mNormals.data(), mesh->mNormals, mesh->mNumVertices * normalSize);
	}
	else
	{
		computeNormals();
	}
}
#endif

void Mesh::computeBoundingBox()
{
    glm::vec3 min(INFINITY);
    glm::vec3 max(-INFINITY);
    for(glm::vec3& vert : mVertices)
    {
        min.x = glm::min(min.x, vert.x);
        max.x = glm::max(max.x, vert.x);

        min.y = glm::min(min.y, vert.y);
        max.y = glm::max(max.y, vert.y);

        min.z = glm::min(min.z, vert.z);
        max.z = glm::max(max.z, vert.z);
    }
    mBBox = BoundingBox(min, max);
}

void Mesh::computeNormals()
{
    mNormals.clear();
    mNormals.assign(mVertices.size(), glm::vec3(0.0f));

    for (int i = 0; i < mIndices.size(); i += 3)
    {
        const glm::vec3 v1 = mVertices[mIndices[i]];
        const glm::vec3 v2 = mVertices[mIndices[i + 1]];
        const glm::vec3 v3 = mVertices[mIndices[i + 2]];
        const glm::vec3 normal = glm::normalize(glm::cross(v2 - v1, v3 - v1));

        mNormals[mIndices[i]] += glm::acos(glm::dot(glm::normalize(v2 - v1), glm::normalize(v3 - v1))) * normal;
        mNormals[mIndices[i + 1]] += glm::acos(glm::dot(glm::normalize(v1 - v2), glm::normalize(v3 - v2))) * normal;
        mNormals[mIndices[i + 2]] += glm::acos(glm::dot(glm::normalize(v1 - v3), glm::normalize(v2 - v3))) * normal;
    }

    for(glm::vec3& n : mNormals)
    {
        n = glm::normalize(n);
    }
}

void Mesh::applyTransform(glm::mat4 trans)
{
    for(glm::vec3& vert : mVertices)
    {
        vert = glm::vec3(trans * glm::vec4(vert, 1.0));
    }

    computeBoundingBox();
}
}
