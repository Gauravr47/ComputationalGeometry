#pragma once
#include<glad/glad.h>
#include<GLFW/glfw3.h>
#include<iostream>
#include<math.h>
#include <string>
#include <fstream>
#include <sstream>
#include "primitives.h"



//variables
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

//function definitions
void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow* window);

//shader class
using namespace std;
class Shader {
public:
	unsigned int ID;
	Shader(const char* vertexPath, const char* fragmentPath);
	void use();
	void setBool(const string& attributeName, bool value) const;
	void setFloat(const string& attributeName, float value) const;
	void setInt(const string& attributeName, int value) const;
	void setVec4(const string& attributeName, float value[4]) const;
};