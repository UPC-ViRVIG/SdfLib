#include "IShader.h"
#include <iostream>
#include <fstream>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <filesystem>

#define CHECK_DEBUG_DIRECTORY
#ifdef CHECK_DEBUG_DIRECTORY
const std::string SHADER_PATH_DEBUG = "../src/render_engine/shaders/";
#endif

IShader::IShader(const std::string& vertexShaderName, const std::string& fragmentShaderName)
{
    unsigned long length;
	//load the vertex shader 
	unsigned int vertexShaderId = glCreateShader(GL_VERTEX_SHADER);

	char* fileShader = loadFromFile(vertexShaderName, &length);
	if (fileShader == nullptr) {
#ifdef CHECK_DEBUG_DIRECTORY
		std::filesystem::path p(vertexShaderName);
		fileShader = loadFromFile(SHADER_PATH_DEBUG + p.filename().string(), &length);
		if (fileShader == nullptr)
			std::cout << "File " << vertexShaderName << " not found" << std::endl;
#else
		std::cout << "File " << vertexShaderName << " not found" << std::endl;
#endif
	}

	glShaderSource(vertexShaderId, 1, &fileShader, NULL);
	glCompileShader(vertexShaderId);

	int success;
	glGetShaderiv(vertexShaderId, GL_COMPILE_STATUS, &success);
	if (!success) {
		char infoLog[512];
		glGetShaderInfoLog(vertexShaderId, 512, NULL, infoLog);
		std::cout << "-> Vertex Shader error ( " << vertexShaderName << " ):" << std::endl;
		std::cout << infoLog << std::endl;
	}

	delete[] fileShader;

	//load the fragment shader
	unsigned int fragmentShaderId = glCreateShader(GL_FRAGMENT_SHADER);

	fileShader = loadFromFile(fragmentShaderName, &length);
	if (fileShader == nullptr) {
#ifdef CHECK_DEBUG_DIRECTORY
		std::filesystem::path p(fragmentShaderName);
		fileShader = loadFromFile(SHADER_PATH_DEBUG + p.filename().string(), &length);
		if (fileShader == nullptr)
			std::cout << "File " << fragmentShaderName << " not found" << std::endl;
#else
		std::cout << "File " << fragmentShaderName << " not found" << std::endl;
#endif
	}

	glShaderSource(fragmentShaderId, 1, &fileShader, NULL);
	glCompileShader(fragmentShaderId);

	glGetShaderiv(fragmentShaderId, GL_COMPILE_STATUS, &success);
	if (!success) {
		char infoLog[512];
		glGetShaderInfoLog(fragmentShaderId, 512, NULL, infoLog);
		std::cout << "-> Fragment Shader error ( " << fragmentShaderName << " ):" << std::endl;
		std::cout << infoLog << std::endl;
	}

	//create the program
	programId = glCreateProgram();

	glAttachShader(programId, vertexShaderId);
	glAttachShader(programId, fragmentShaderId);
	glLinkProgram(programId);

	glDeleteShader(vertexShaderId);
	glDeleteShader(fragmentShaderId);

	glGetProgramiv(programId, GL_LINK_STATUS, &success);
	if (!success) {
		char infoLog[512];
		glGetShaderInfoLog(fragmentShaderId, 512, NULL, infoLog);
		std::cout << "-> Link Shader error" << std::endl;
		std::cout << infoLog << std::endl;
	}

	//get standard attributes
	projectionViewModelMatrixLocation = glGetUniformLocation(programId, "projectionViewModelMatrix");
	projectionMatrixLocation = glGetUniformLocation(programId, "projectionMatrix");
	viewModelMatrixLocation = glGetUniformLocation(programId, "viewModelMatrix");
	invViewModelMatrixLocation = glGetUniformLocation(programId, "invViewModelMatrix");
	modelMatrixLocation = glGetUniformLocation(programId, "modelMatrix");
	normalViewModelMatrixLocation = glGetUniformLocation(programId, "normalViewModelMatrix");
	normalModelMatrixLocation = glGetUniformLocation(programId, "normalModelMatrix");
}

IShader::~IShader()
{
    glDeleteProgram(programId);
}

void IShader::setMatrices(const glm::mat4x4& projection, const glm::mat4x4& view, const glm::mat4x4& model)
{
    glm::mat4x4 res = model;
	if (modelMatrixLocation != -1) {
		glUniformMatrix4fv(modelMatrixLocation, 1, GL_FALSE, glm::value_ptr(res));
	}
	if (normalModelMatrixLocation != -1) {
		glUniformMatrix3fv(normalModelMatrixLocation, 1, GL_FALSE, glm::value_ptr(glm::inverseTranspose(glm::mat3(res))));
	}
	if (projectionMatrixLocation != -1) {
		glUniformMatrix4fv(projectionMatrixLocation, 1, GL_FALSE, glm::value_ptr(projection));
	}

	res = view * res;
	if (viewModelMatrixLocation != -1) {
		glUniformMatrix4fv(viewModelMatrixLocation, 1, GL_FALSE, glm::value_ptr(res));
	}

	if (invViewModelMatrixLocation != -1) {
		glm::mat4x4 invRes = glm::inverse(res);
		glUniformMatrix4fv(invViewModelMatrixLocation, 1, GL_FALSE, glm::value_ptr(invRes));
	}

	if (normalViewModelMatrixLocation != -1) {
		glUniformMatrix3fv(normalViewModelMatrixLocation, 1, GL_FALSE, glm::value_ptr(glm::inverseTranspose(glm::mat3(res))));
	}

	res = projection * res;
	if (projectionViewModelMatrixLocation != -1) {
		glUniformMatrix4fv(projectionViewModelMatrixLocation, 1, GL_FALSE, glm::value_ptr(res));
	}
}

char* IShader::loadFromFile(std::string path, unsigned long* length)
{
    std::ifstream file;
	file.open(path, std::ios_base::in | std::ios_base::binary);
	if (!file.good()) return nullptr;
	file.seekg(0, std::ios::end);
	*length = file.tellg();
	(*length)++;
	char* ret = new char[*length];
	file.seekg(0, std::ios::beg);
	file.read(ret, *length);
	file.close();
	ret[(*length) - 1] = 0;
	return ret;
}