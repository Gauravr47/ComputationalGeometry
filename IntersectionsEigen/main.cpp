#include "primitives.h"
#include "OpenGLFunctions.h"
using namespace std;

int main() {
	try {
		Solid s("testFiles/cube_3_hollow.stl");
		std::ofstream ofs("test.log");
		std::cout << "\n Archiving";
		// save data to archive
		{
			boost::archive::text_oarchive oa(ofs);
			// write class instance to archive
			oa << s;
			// archive and stream closed when destructors are called
		}


		//open GL code
		glfwInit();
		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
		glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
		glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

		GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", NULL, NULL);
		if (window == NULL)
		{
			std::cout << "Failed to create GLFW window" << std::endl;
			glfwTerminate();
			return -1;
		}
		glfwMakeContextCurrent(window);

		if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
		{
			std::cout << "Failed to initialize GLAD" << std::endl;
			return -1;
		}

		glViewport(0, 0, 800, 600);

		glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

		Shader shaderProgram("shader.vs", "shader.fs");

		//add the input to VBO
		unsigned int VBO[2], VAO[2];
		glGenVertexArrays(2, VAO);
		glGenBuffers(2, VBO);

		vector<point3d> intersections;
		point3d planeNormal(0, 0, 1);
		point3d planePoint(0, 0, 0.5);
		vector<edge> edges = std::move(s.getEdges());
		slicingPlaneIntersection(planeNormal, planePoint, edges , intersections);
		std::cout << "\n Done intersecting";
		//intersections.erase(unique(intersections.begin(), intersections.end()), intersections.end());


		if (!intersections.empty()) {
			glBindVertexArray(VAO[0]);
			glBindBuffer(GL_ARRAY_BUFFER, VBO[0]);
			glBufferData(GL_ARRAY_BUFFER, intersections.size() * 3 * sizeof(float), &intersections.front(), GL_STATIC_DRAW);
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);	// Vertex attributes stay the same
			glEnableVertexAttribArray(0);
		}

		vector<point3d> hull;
		boost::geometry::convex_hull(intersections, hull);

		std::cout << "\n Hull";
		for (auto const intS : hull) {
			std::cout << "\n\n" << intS;
		}

		if (!hull.empty())
		{
			glBindVertexArray(VAO[1]);
			glBindBuffer(GL_ARRAY_BUFFER, VBO[1]);
			glBufferData(GL_ARRAY_BUFFER, sizeof(hull) * 3 * sizeof(float), &hull.front(), GL_STATIC_DRAW);
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);	// Vertex attributes stay the same
			glEnableVertexAttribArray(0);
		}

		while (false && !glfwWindowShouldClose(window))
		{
			// input
			// -----
			processInput(window);

			// render
			// ------
			glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
			glClear(GL_COLOR_BUFFER_BIT);

			float color[4] = { 1.0f, 0.5f, 0.2f, 1.0f };
			glUseProgram(shaderProgram.ID);
			shaderProgram.use();
			shaderProgram.setVec4("outColor", color);
			// draw first triangle using the data from the first VAO
			glBindVertexArray(VAO[0]);
			glDrawArrays(GL_POINTS, 0, intersections.size());
			glPointSize(5);

			float colorE[4] = { 0.0f, 0.5f, 0.2f, 1.0f };
			shaderProgram.setVec4("outColor", colorE);
			glBindVertexArray(VAO[1]);
			glDrawArrays(GL_LINE_STRIP, 0, hull.size());
			glLineWidth(2);

			// -------------------------------------------------------------------------------
			glfwSwapBuffers(window);
			glfwPollEvents();
		}


		// ... some time later restore the class instance to its orginal state
		std::cout << "\n Retreiving archive";
		Solid newg;
		{
			// create and open an archive for input
			std::ifstream ifs("test.log");
			boost::archive::text_iarchive ia(ifs);
			// read class state from archive
			ia >> newg;
			// archive and stream closed when destructors are called
		}


	}
	catch (exception& e) {
		std::cerr << e.what();
	}

	return 0;
}