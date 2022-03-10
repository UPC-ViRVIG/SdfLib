#ifndef SHADER_H
#define SHADER_H

#include "IShader.h"

const std::string SHADER_PATH = "../src/render_engine/shaders/";
// const std::string SHADER_PATH = "./shaders/";

template <class T>
class Shader : public IShader 
{
public:
	Shader(const std::string& vertexShaderName, const std::string& fragmentShaderName) : IShader(vertexShaderName, fragmentShaderName) {}
	static T* getInstance() {
		if (instance == nullptr) instance = new T();
		return instance;
	}

	unsigned int getVertexShader() { return vertexShaderId; }
	unsigned int getFragmentShader() { return fragmentShaderId; }

private:
	static T* instance;
};

template <class T>
T* Shader<T>::instance = nullptr;

#endif