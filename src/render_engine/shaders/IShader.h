#ifndef ISHADER_H
#define ISHADER_H

#include <string>
#include <glm/glm.hpp>
#include <glad/glad.h>

class IShader {
public:
	IShader(const std::string& vertexShaderName, const std::string& fragmentShaderName);
	~IShader();
	unsigned int getProgramId() { return programId; }
	void setMatrices(const glm::mat4x4& projection, const glm::mat4x4& view, const glm::mat4x4& model);
	virtual void bind() {}
private:
	unsigned int projectionViewModelMatrixLocation;
	unsigned int projectionMatrixLocation;
	unsigned int viewModelMatrixLocation;
	unsigned int invViewModelMatrixLocation;
	unsigned int modelMatrixLocation;
	unsigned int normalViewModelMatrixLocation;
	unsigned int normalModelMatrixLocation;

	unsigned int programId;

	char* loadFromFile(std::string path, unsigned long* length);
};

#endif // ISHADER_H