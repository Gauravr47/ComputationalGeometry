#include"OpenGLFunctions.h"
#include"convexHull.h"

// Main function
// Function to generate a random double between fMin and fMax
double fRand(double fMin, double fMax) {
    return fMin + (static_cast<double>(rand()) / RAND_MAX) * (fMax - fMin);
}

// Function to generate a random integer between fMin and fMax
double IRand(double fMin, double fMax) {
    return fMin + (rand() % static_cast<int>(fMax - fMin + 1));
}

std::vector<Point> getPointSet(int num_of_points) {
    // Generate random points and insert into the set to ensure uniqueness
    set<Point> pointSet;
    vector<Point> ans;
   while( pointSet.size() < num_of_points) {
        double x = fRand(-1.0, 1.0);
        double y = fRand(-1.0, 1.0);
        Point temp(x, y);
        pointSet.insert(temp);
    }
    ans.assign(pointSet.begin(), pointSet.end());
    return ans;
}

int main() {
    // Number of points
    const int np = 100;

    // Vector to store unique points
    std::vector<Point> input;

    // Vector to store all legitimate edges
    std::vector<Edge> allEdges;

    // Vector to store convex hull points
    std::vector<Point> hull;

    // Set to ensure unique points
    std::set<Point> pointSet;

    //open GL code
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    //glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

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

    //get input of points
    input = getPointSet(np);

    //add the input to VBO
    unsigned int VBO[2], VAO[2], EBO[1];
    glGenVertexArrays(2, VAO);
    glGenBuffers(2, VBO);

    GLfloat vertices[np*3];
    GLfloat edges[np * 3];
    // Print the generated points
    std::cout << "Generated Points:" << std::endl;
    for (int i = 0; i < np; i++) {

        std::cout << input[i] << std::endl;
        vertices[3 * i + 0] = input[i].x;
        vertices[3 * i + 1] = input[i].y;
        vertices[3 * i + 2] = input[i].z;
    }
    glBindVertexArray(VAO[0]);
    glBindBuffer(GL_ARRAY_BUFFER, VBO[0]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);	// Vertex attributes stay the same
    glEnableVertexAttribArray(0);

    // Compute convex hull using the naive approach
    allEdges = ConvexHull(input);

    // Convert edges to polygon points
    hull = FromEdgesToPolyPoints(allEdges);

  
    // Print the resulting convex hull
    std::cout << "Convex Hull (Naive Approach):" << std::endl;
    for (const auto& point : hull) {
        std::cout << point << std::endl;
    }

    // Compute convex hull using the quick approach
    auto hull2 = ConvexHullFast(input);

    // Print the resulting convex hull
    std::cout << "Convex Hull (Quick Approach):" << std::endl;
    for (int i = 0; i < hull2.size(); i++) {

        std::cout << hull2[i] << std::endl;
        edges[3 * i + 0] = hull2[i].x;
        edges[3 * i + 1] = hull2[i].y;
        edges[3 * i + 2] = hull2[i].z;
    }

    glBindVertexArray(VAO[1]);
    glBindBuffer(GL_ARRAY_BUFFER, VBO[1]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(edges), edges, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);	// Vertex attributes stay the same
    glEnableVertexAttribArray(0);

    while (!glfwWindowShouldClose(window))
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
        glDrawArrays(GL_POINTS, 0, np);
        glPointSize(5);

        float colorE[4] = { 0.0f, 0.5f, 0.2f, 1.0f };
        shaderProgram.setVec4("outColor", colorE);
        glBindVertexArray(VAO[1]);
        glDrawArrays(GL_LINE_STRIP, 0, hull2.size());
        glLineWidth(2);
        // then we draw the second triangle using the data from the second VAO
        //glBindVertexArray(VAO[1]);
        //glDrawArrays(GL_LINES, 0, 3);

        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        // -------------------------------------------------------------------------------
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

 

    return 0;
}
