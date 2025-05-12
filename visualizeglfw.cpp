#include "visualizeglfw.hpp"
#include "geometry_loader.hpp"
#include "ray.hpp"
#include "shaders.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>  // For transformations (e.g., glm::translate)
#include <glm/gtc/type_ptr.hpp>          // For glm::value_ptr

#define GLAD_GL_IMPLEMENTATION
#include <glad/glad.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>


struct RayVertex {
    double x, y, z;  // Position
    float r, g, b;  // Color (optional)
};
 
// void draw(const Ray& ray) {
//     glBegin(GL_LINES);
//         glColor3f(1.0f, 0.0f, 0.0f);  // Red color
//         glVertex3f(ray.origin.x, ray.origin.y, ray.origin.z);
//         // Extend the ray by scaling the direction (e.g., 5 units)
//         glVertex3f(ray.end.x, ray.end.y, ray.end.z);
//     glEnd();
// }

void visualizeWithGLFW(GeometryLoader& geometry) {
    int width = 1000;
    int height = 1000;
     
    // glfwSetErrorCallback(error_callback);
 
    if (!glfwInit())
    {
        exit(EXIT_FAILURE);
    }
 
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
 
    GLFWwindow* window = glfwCreateWindow(width, height, "OpenGL OpticSim", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }
    glfwGetWindowSize(window, &width, &height);
   
    glfwMakeContextCurrent(window);
    gladLoadGL();
    glfwSwapInterval(1);

    // Convert rays to vertices (2 vertices per ray: start + end)
    std::vector<RayVertex> vertices;
    for (const Ray& ray : geometry.rays) {
        // Origin (white)
        vertices.push_back(RayVertex{ray.origin.x, ray.origin.y, ray.origin.z, 1.0f, 1.0f, 1.0f});
        // Endpoint (red, scaled by length)
        float length = 5.0f;  // Adjust as needed
        vertices.push_back(RayVertex{ray.end.x, ray.end.y, ray.end.z,
            1.0f, 0.0f, 0.0f  // RGB color
        });
    }

    unsigned int VAO, VBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(RayVertex), vertices.data(), GL_STATIC_DRAW);

    const GLuint vertex_shader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex_shader, 1, &vertex_shader_text, NULL);
    glCompileShader(vertex_shader);
 
    const GLuint fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment_shader, 1, &fragment_shader_text, NULL);
    glCompileShader(fragment_shader);
 
    const GLuint program = glCreateProgram();
    glAttachShader(program, vertex_shader);
    glAttachShader(program, fragment_shader);
    glLinkProgram(program);


    // Position attribute (location = 0)
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // Color attribute (location = 1)
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    glUseProgram(program);  // Compile/link shaders first
    glBindVertexArray(VAO);
    glDrawArrays(GL_LINES, 0, geometry.rays.size() * 2);  // 2 vertices per ray
 
    while (!glfwWindowShouldClose(window))
    {
        glfwGetWindowSize(window, &width, &height);
 
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glm::mat4 model = glm::mat4(1.0f);
        // model = glm::rotate(model, glm::radians(45.0f), glm::vec3(0.0f, 0.0f, 1.0f));

        glm::mat4 view = glm::lookAt(
            glm::vec3(0.0f, -1.0f, 3.0f),  // Camera position
            glm::vec3(0.0f, 0.0f, 0.0f),  // Target
            glm::vec3(0.0f, 1.0f, 0.0f)   // Up vector
        );

        glm::mat4 projection = glm::perspective(
            glm::radians(45.0f), 
            static_cast<float>(width) / height,  // Use dynamic aspect ratio
            0.1f, 
            100.0f
        );

        glm::mat4 mvp = projection * view * model;  // Combine matrices
        glUseProgram(program);
        glUniformMatrix4fv(glGetUniformLocation(program, "MVP"), 1, GL_FALSE, glm::value_ptr(mvp));

        glBindVertexArray(VAO);
        glDrawArrays(GL_LINES, 0, geometry.rays.size() * 2);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteProgram(program);
    glfwTerminate();
 
    glfwDestroyWindow(window);
 
    glfwTerminate();
    exit(EXIT_SUCCESS);

}