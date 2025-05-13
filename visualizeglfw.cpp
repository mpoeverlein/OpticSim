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

struct Vertex {
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec3 color;
};

std::vector<Vertex> createCylinder(
    const Vector& start_, 
    const Vector& end_, 
    float radius, 
    int segments = 16,
    const glm::vec3& color = glm::vec3(1.0f, 0.0f, 0.0f)
)
{
    glm::vec3 start{start_.x, start_.y, start_.z};
    glm::vec3 end{end_.x, end_.y, end_.z};
    std::vector<Vertex> vertices;
    glm::vec3 axis = glm::normalize(end - start);
    float height = glm::length(end - start);

    // Generate two rings of vertices
    for (int i = 0; i <= segments; ++i) {
        float angle = 2.0f * M_PI * i / segments;
        glm::vec3 circleDir(cos(angle), sin(angle), 0.0f);
        glm::vec3 normal = glm::normalize(circleDir);

        // Rotate circleDir to align with the cylinder axis
        glm::vec3 tangent = glm::cross(glm::vec3(0.0f, 0.0f, 1.0f), axis);
        float rotationAngle = acos(glm::dot(glm::vec3(0.0f, 0.0f, 1.0f), axis));
        glm::mat4 rotation = glm::rotate(glm::mat4(1.0f), rotationAngle, tangent);
        circleDir = glm::vec3(rotation * glm::vec4(circleDir, 0.0f));

        // Bottom and top vertices
        vertices.push_back({start + circleDir * radius, circleDir, color});
        vertices.push_back({end + circleDir * radius, circleDir, color});
    }

    return vertices;  // Combine with indices for rendering
}

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
    std::vector<Vertex> vertices;
    // for (const Ray& ray : geometry.rays) {
    //     float r;
    //     if (ray.refractiveIndex == Config::VACUUM_REFRACTIVE_INDEX) {
    //         r = 1.0;
    //     } else {
    //         r = 0.0;
    //     }
    //     // Origin (white)
    //     vertices.push_back(RayVertex{(float) ray.origin.x, (float) ray.origin.y, (float) ray.origin.z, r, 1.0f, 1.0f});
    //     // Endpoint (red, scaled by length)
    //     vertices.push_back(RayVertex{(float) ray.end.x, (float) ray.end.y, (float) ray.end.z,
    //         r, 1.0f, 1.0f  // RGB color
    //     });
    //     // float scale = 5;
    //     // vertices.push_back(RayVertex{float(ray.origin.x + scale * ray.direction.x), float(ray.origin.y + scale * ray.direction.y), float(ray.origin.z + scale * ray.direction.z),
    //     //     1.0f, 0.0f, 0.0f  // RGB color
    //     // });
    // }
    int segments = 16;
    std::vector<unsigned int> indices;
    for (const Ray& ray: geometry.rays) {
        auto cylinderVerts = createCylinder(ray.origin, ray.end, 0.05f, segments=16);
        for (const Vertex& cv: cylinderVerts) {
            vertices.push_back(cv);
        }

        for (int i = 0; i < segments; ++i) {
            unsigned int base = i * 2;
            indices.insert(indices.end(), {base, base + 1, base + 2});
            indices.insert(indices.end(), {base + 1, base + 3, base + 2});
        }
    }

    unsigned int VAO, VBO, EBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

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


    // Position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
    glEnableVertexAttribArray(0);

    // Normal attribute (for lighting)
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));
    glEnableVertexAttribArray(1);

    // Color attribute
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, color));
    glEnableVertexAttribArray(2);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    glUseProgram(program);  // Compile/link shaders first
    glBindVertexArray(VAO);
    // glDrawArrays(GL_LINES, 0, vertices.size());  // 2 vertices per ray
    glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);

 
    while (!glfwWindowShouldClose(window))
    {
        glfwGetWindowSize(window, &width, &height);
 
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

        glm::mat4 model = glm::mat4(1.0f);
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
        // glDrawArrays(GL_LINES, 0, vertices.size());
        glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);


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