#ifndef SHADER_H
#define SHADER_H

#include "IShader.h"

const std::string SHADER_PATH = "./shaders/";

template <class T>
class Shader : public IShader 
{
public:
	Shader(const std::string& vertexShaderName, const std::string& fragmentShaderName) : IShader(vertexShaderName, fragmentShaderName) {}
	Shader(const std::string& vertexShaderName, const std::string& vertexShaderHeader, const std::string& fragmentShaderName, const std::string& fragmentShaderHeader) :
		IShader(vertexShaderName, vertexShaderHeader, fragmentShaderName, fragmentShaderHeader) {}
	static T* getInstance() {
		if (instance == nullptr) instance = new T();
		return instance;
	}

private:
	static T* instance;
};

template <class T>
T* Shader<T>::instance = nullptr;

#endif