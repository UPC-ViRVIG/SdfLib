#ifndef RENDER_MESH_H
#define RENDER_MESH_H

#include <glad/glad.h>
#include <vector>

#include "System.h"
#include "shaders/IShader.h"

class RenderMesh : public System
{
public:
    struct VertexParameterLayout {
        GLenum type;
        int size;
        VertexParameterLayout() {}
        VertexParameterLayout(GLenum type, int size) : type(type), size(size) {}
    };

    ~RenderMesh();
    void start() override;
    void draw(Camera* camera) override;
	void drawGui() override;

	uint32_t setVertexData(std::vector<VertexParameterLayout> parameters, void* data, size_t numElements);
    void setVertexData(uint32_t bufferId, void* data, size_t numElements);
    void setIndexData(std::vector<unsigned int>& indices);
	void setIndexData(unsigned int* data, size_t numElements);
	void setIndexData(unsigned int* data, size_t numElements, GLenum mode);
	void setDrawMode(GLenum mode) { mDrawMode = mode; }
    void setDataMode(GLenum mode) { mFormat = mode; }
	void setShader(IShader* shader) { mShader = shader; }
	void drawWireframe(bool b) { mPrintWireframe = b; }
	bool isDrawingWireframe() { return mPrintWireframe; }
	void drawSurface(bool b) { mPrintSurface = b; }
	bool isDrawingSurface() { return mPrintSurface; }

    const glm::mat4x4& getTransform() const { return mTransform; }
    void setTransform(glm::mat4x4 transfrom) { mTransform = transfrom; }

private:
    struct BufferData
    {
        BufferData(unsigned int VBO, size_t elementsSize) : 
            VBO(VBO), elementsSize(elementsSize) {} 
        unsigned int VBO;
        size_t elementsSize;
    };

    bool mMeshAllocated = false;
    unsigned int mVAO;        
    std::vector<BufferData> mBuffersData;
    bool mHasElementBuffer = false;
    unsigned int mEBO;

    uint32_t mNextAttributeIndex = 0;
	
    size_t mIndexArraySize = 0; // Number of inices
    size_t mDataArraySize = 0;

    bool mPrintSurface = true;
    bool mPrintWireframe = false;

    GLenum mFormat = GL_TRIANGLES;
    GLenum mDrawMode = GL_FILL;

    IShader* mShader;

    glm::mat4x4 mTransform = glm::mat4x4(1.0f);
};

#endif // RENDER_MESH_H