#include"OpenGLFunctions.h"

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	glViewport(0, 0, width, height);
}

void processInput(GLFWwindow* window)
{
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);
}

//shader class
Shader::Shader(const char* vertexPath, const char* fragmentPath) {
	//define variables
	ifstream vertexShaderFile, fragmentShaderFile;
	string vertexShaderSource, fragmentShaderSource;

	vertexShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
	fragmentShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);

	int success;
	char infoLog[512];
	try {
		//from paths read files
		vertexShaderFile.open(vertexPath);
		fragmentShaderFile.open(fragmentPath);

		//from files read functions in stream
		stringstream vertexStream, fragmentStream;
		vertexStream << vertexShaderFile.rdbuf();
		fragmentStream << fragmentShaderFile.rdbuf();

		//close files
		vertexShaderFile.close();
		fragmentShaderFile.close();

		//convert stream to string
		vertexShaderSource = vertexStream.str();
		fragmentShaderSource = fragmentStream.str();

	}
	catch (ifstream::failure e)
	{
		cout << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ" << endl;
	}
	const char* vertexCode = vertexShaderSource.c_str();
	const char* fragmentCode = fragmentShaderSource.c_str();
	//from functions create shaders and compile them
	unsigned int vertexShader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vertexShader, 1, &vertexCode, NULL);
	glCompileShader(vertexShader);
	// check for shader compile errors

	glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
	if (!success)
	{
		glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
		std::cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" << infoLog << std::endl;
	}


	unsigned int fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(fragmentShader, 1, &fragmentCode, NULL);
	glCompileShader(fragmentShader);
	glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);

	if (!success) {
		glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
		std::cout << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n" << infoLog << std::endl;
	}

	//create a shader program and attach shaders

	ID = glCreateProgram();
	glAttachShader(ID, vertexShader);
	glAttachShader(ID, fragmentShader);
	glLinkProgram(ID);

	glGetProgramiv(ID, GL_LINK_STATUS, &success);
	if (!success) {
		glGetProgramInfoLog(ID, 512, NULL, infoLog);
		std::cout << "ERROR::PROGRAM::SHADER::LINKING_FAILED\n" << infoLog << std::endl;
	}
	//delete shaders
	glDeleteShader(vertexShader);
	glDeleteShader(fragmentShader);

};


void Shader::use() {
	glUseProgram(ID);
};

void Shader::setBool(const string& attributeName, bool value) const {
	glUniform1i(glGetUniformLocation(ID, attributeName.c_str()), (int)value);
};

void Shader::setInt(const string& attributeName, int value) const {
	glUniform1i(glGetUniformLocation(ID, attributeName.c_str()), value);
};


void Shader::setFloat(const string& attributeName, float value) const {
	glUniform1f(glGetUniformLocation(ID, attributeName.c_str()), value);
};

void Shader::setVec4(const string& attributeName, float value[4]) const {
	glUniform4f(glGetUniformLocation(ID, attributeName.c_str()), value[0], value[1], value[2], value[3]);
}