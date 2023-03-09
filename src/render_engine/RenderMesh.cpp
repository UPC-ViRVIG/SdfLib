
#include "RenderMesh.h"
#include <iostream>
#include <imgui.h>
#include "Camera.h"
#include "shaders/GridShader.h"

RenderMesh::~RenderMesh()
{
    glDeleteVertexArrays(1, &mVAO);
	for(BufferData& b : mBuffersData)
	{
		glDeleteBuffers(1, &b.VBO);
	}
	if(mHasElementBuffer) glDeleteBuffers(1, &mEBO);
}

void RenderMesh::start()
{
    glGenVertexArrays(1, &mVAO);
	glBindVertexArray(mVAO);
	glBindVertexArray(0);
}

void RenderMesh::draw(Camera* camera)
{
    if (!mMeshAllocated) return;

	glBindVertexArray(mVAO);

	if (mPrintSurface) {
		glUseProgram(mShader->getProgramId());

		mShader->setMatrices(camera->getProjectionMatrix(), camera->getViewMatrix(), mTransform);
		mShader->bind();

		//draw
		if (mDrawMode != GL_FILL) glPolygonMode(GL_FRONT_AND_BACK, mDrawMode);
		
		if(mHasElementBuffer)
		{
			glDrawElements(mFormat, mIndexArraySize, GL_UNSIGNED_INT, 0);
		}
		else
		{
			glDrawArrays(mFormat, 0, mDataArraySize);
		}

		if (mDrawMode != GL_FILL) glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	}


	if (mPrintWireframe) {

		glLineWidth(3);

		IShader* mat = Shader<GridShader>::getInstance();

		glUseProgram(mat->getProgramId());

		mat->setMatrices(camera->getProjectionMatrix(), camera->getViewMatrix(), mTransform);
		mat->bind();

		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

		glDepthFunc(GL_LEQUAL);

		if(mHasElementBuffer)
		{
			glDrawElements(mFormat, mIndexArraySize, GL_UNSIGNED_INT, 0);
		}
		else
		{
			glDrawArrays(mFormat, 0, mDataArraySize);
		}

		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

		glDepthFunc(GL_LESS);
	}
	
	glBindVertexArray(0);
}

void RenderMesh::drawGui()
{
	ImGui::Spacing();
	ImGui::Separator();
	ImGui::Text((systemName == "") ? "RenderMesh" : systemName.c_str());
    ImGui::Checkbox("Draw Wireframe", &mPrintWireframe);
	ImGui::Checkbox("Draw Surface", &mPrintSurface);
}

int getSize(GLenum type) {
	switch (type) {
	case GL_FLOAT:
		return 4;
	}
	return 0;
}


uint32_t RenderMesh::setVertexData(std::vector<VertexParameterLayout> parameters, void* data, size_t numElements)
{
	mDataArraySize = numElements;

	glBindVertexArray(mVAO);
	unsigned int VBO;
	glGenBuffers(1, &VBO);

	// Calculate the total size
	size_t stripSize = 0;
	for(VertexParameterLayout& parameter : parameters)
    {
		stripSize += parameter.size * getSize(parameter.type);
	}

	mBuffersData.push_back(BufferData(VBO, stripSize));

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, numElements * stripSize, data, GL_STATIC_DRAW);

	// Set the vertex parameters
	int currentSize = 0;
	for (uint32_t i = 0; i < parameters.size(); i++) {
		glVertexAttribPointer(mNextAttributeIndex, parameters[i].size, parameters[i].type, GL_FALSE, stripSize, (void*) currentSize);
		glEnableVertexAttribArray(mNextAttributeIndex++);
		currentSize += parameters[i].size * getSize(parameters[i].type);
	}

	glBindVertexArray(0);

	mMeshAllocated = true;

	return mBuffersData.size() - 1;
}

void RenderMesh::setVertexData(uint32_t bufferId, void* data, size_t numElements)
{
	mDataArraySize = numElements;

	glBindVertexArray(mVAO);

	glBindBuffer(GL_ARRAY_BUFFER, mBuffersData[bufferId].VBO);
	glBufferData(GL_ARRAY_BUFFER, numElements * mBuffersData[bufferId].elementsSize, data, GL_STATIC_DRAW);

	glBindVertexArray(0);
}

void RenderMesh::setIndexData(std::vector<unsigned int>& indices)
{
	setIndexData(indices.data(), indices.size());
}

void RenderMesh::setIndexData(unsigned int* data, size_t numElements)
{
    setIndexData(data, numElements, GL_TRIANGLES);
}

void RenderMesh::setIndexData(unsigned int* data, size_t numElements, GLenum mode)
{
	glBindVertexArray(mVAO);

	if(!mHasElementBuffer)
	{
		glGenBuffers(1, &mEBO);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mEBO);
		mHasElementBuffer = true;
	}

    mIndexArraySize = numElements;
	mFormat = mode;

	glBufferData(GL_ELEMENT_ARRAY_BUFFER, mIndexArraySize * sizeof(unsigned int), data, GL_STATIC_DRAW);

	glBindVertexArray(0);
}