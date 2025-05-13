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
    float x, y, z;  // Position
    float r, g, b;  // Color (optional)
};

class Camera {
public:
    glm::vec3 position;
    glm::vec3 front;
    glm::vec3 up;
    glm::vec3 right;
    glm::vec3 worldUp;
    
    float yaw = 90.0f;   // Rotation around Z axis (starts looking along -Y)
    float pitch = 0.0f;   // Rotation around X axis
    float speed = 0.05f;
    float sensitivity = 0.1f;

    Camera(glm::vec3 pos = glm::vec3(0.0f, -3.0f, 0.0f), 
           glm::vec3 up = glm::vec3(0.0f, 0.0f, 1.0f)) 
        : position(pos), worldUp(up) {
        updateVectors();
    }

    glm::mat4 getViewMatrix() {
        return glm::lookAt(position, position + front, up);
    }

    void updateVectors() {
        // Calculate new front vector
        glm::vec3 newFront;
        newFront.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
        newFront.z = sin(glm::radians(pitch));
        newFront.y = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
        front = glm::normalize(newFront);
        
        right = glm::normalize(glm::cross(front, worldUp));
        up = glm::normalize(glm::cross(right, front));
    }
};
 

int width = 1000;
int height = 1000;
Camera camera;

bool firstMouse = true;
float lastX = width / 2.0f;
float lastY = height / 2.0f;

// Keyboard callback
void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        switch (key) {
            case GLFW_KEY_W: camera.position += camera.front * camera.speed; break;
            case GLFW_KEY_S: camera.position -= camera.front * camera.speed; break;
            case GLFW_KEY_A: camera.position -= camera.right * camera.speed; break;
            case GLFW_KEY_D: camera.position += camera.right * camera.speed; break;
        }
    }
}

// Mouse callback
void mouseCallback(GLFWwindow* window, double xpos, double ypos) {
    if (firstMouse) {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xoffset = xpos - lastX;
    // float yoffset = lastY - ypos; // Reversed since y-coordinates go from bottom to top
    float yoffset = ypos - lastY;
    lastX = xpos;
    lastY = ypos;

    xoffset *= camera.sensitivity;
    yoffset *= camera.sensitivity;

    camera.yaw += xoffset;
    camera.pitch += yoffset;

    // Constrain pitch to avoid screen flipping
    if (camera.pitch > 89.0f) camera.pitch = 89.0f;
    if (camera.pitch < -89.0f) camera.pitch = -89.0f;

    camera.updateVectors();
}

void visualizeWithGLFW(GeometryLoader& geometry) {

    Vector p(0,-1,3);
    double zoomFactor = 1.;
     
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
    glfwSetKeyCallback(window, keyCallback);
    glfwSetCursorPosCallback(window, mouseCallback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED); // Hide and capture mouse
   
    glfwMakeContextCurrent(window);
    gladLoadGL();
    glEnable(GL_DEPTH_TEST);  // Add this after gladLoadGL()

    glfwSwapInterval(1);


    // Convert rays to vertices (2 vertices per ray: start + end)
    std::vector<RayVertex> vertices;
    for (const Ray& ray : geometry.rays) {
        // Origin (white)
        vertices.push_back(RayVertex{(float) ray.origin.x, (float) ray.origin.y, (float) ray.origin.z, 1.0f, 1.0f, 1.0f});
        // Endpoint (red, scaled by length)
        // vertices.push_back(RayVertex{(float) ray.end.x, (float) ray.end.y, (float) ray.end.z,
        //     1.0f, 0.0f, 0.0f  // RGB color
        // });
        float scale = 5;
        vertices.push_back(RayVertex{float(ray.origin.x + scale * ray.direction.x), float(ray.origin.y + scale * ray.direction.y), float(ray.origin.z + scale * ray.direction.z),
            1.0f, 0.0f, 0.0f  // RGB color
        });
        std::cout << ray << "\n";
        std::cout << vertices[vertices.size()-2].z << "\n";
    }

    vertices.push_back(RayVertex{0,0,0,1,1,1});
    vertices.push_back(RayVertex{1,0,0,1,0,0});
    vertices.push_back(RayVertex{0,0,0,1,1,1});
    vertices.push_back(RayVertex{0,1,0,0,1,0});
    vertices.push_back(RayVertex{0,0,0,1,1,1});
    vertices.push_back(RayVertex{0,0,1,0,0,1});
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
    glDrawArrays(GL_LINES, 0, vertices.size());  // 2 vertices per ray
    glPointSize(10.0f);
    glDrawArrays(GL_POINTS, 0, vertices.size());
 
    while (!glfwWindowShouldClose(window))
    {
        glfwGetWindowSize(window, &width, &height);
 
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

        glm::mat4 model = glm::mat4(1.0f);

        // glm::mat4 view = glm::lookAt(
        //     glm::vec3(p.x, p.y, p.z),  // Camera position
        //     // glm::vec3(0.0f, -1.0f, 3.0f),  // Adjusted camera position (z > 0)
        //     glm::vec3(p.x, p.y+1, p.z-3),  // Target
        //     // glm::vec3(0.0f, 0.0f, 0.0f),
        //     glm::vec3(0.0f, 1.0f, 0.0f)   // Up vector
        // );

        // glm::mat4 projection = glm::perspective(
        //     glm::radians(45.0f), 
        //     static_cast<float>(width) / height,  // Use dynamic aspect ratio
        //     0.1f, 
        //     100.0f
        // );

        // glm::mat4 projection = glm::ortho(
        //     0.0f, 800.0f, 0.0f, 600.0f, 0.1f, 100.0f
        // );

        // glm::mat4 mvp = projection * view * model;  // Combine matrices

        glm::mat4 view = camera.getViewMatrix();
        glm::mat4 projection = glm::perspective(
            glm::radians(45.0f), 
            (float)width / (float)height, 
            0.01f, 
            1000.0f
        );
        glm::mat4 mvp = projection * view * glm::mat4(1.0f);
        glUseProgram(program);
        glUniformMatrix4fv(glGetUniformLocation(program, "MVP"), 1, GL_FALSE, glm::value_ptr(mvp));

        glBindVertexArray(VAO);
        glDrawArrays(GL_LINES, 0, vertices.size());
        glPointSize(10.0f);
        glDrawArrays(GL_POINTS, 0, vertices.size());

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