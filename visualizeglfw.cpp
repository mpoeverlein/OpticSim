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


std::vector<Vertex> createCylinder(
    const Vector& start_, 
    const Vector& end_, 
    float radius, 
    int segments = 16,
    const glm::vec3& color = glm::vec3(1.0f, 1.0f, 1.0f)
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
        // glm::vec3 normal{1,0,0};

        // Rotate circleDir to align with the cylinder axis
        glm::vec3 tangent = glm::cross(glm::vec3(0.0f, 0.0f, 1.0f), axis);
        float rotationAngle = acos(glm::dot(glm::vec3(0.0f, 0.0f, 1.0f), axis));
        glm::mat4 rotation = glm::rotate(glm::mat4(1.0f), rotationAngle, tangent);
        circleDir = glm::vec3(rotation * glm::vec4(circleDir, 0.0f));

        // Bottom and top vertices
        vertices.push_back({start + circleDir * radius, circleDir, color, 1.0f});
        vertices.push_back({end + circleDir * radius, circleDir, color, 1.0f});
    }

    return vertices;  // Combine with indices for rendering
}

glm::mat4 orientSphere(const glm::vec3& targetDirection) {
    const glm::vec3 originalUp(0.0f, 0.0f, 1.0f); // Assuming Z-up
    const glm::vec3 targetDir = glm::normalize(targetDirection);
    
    // Handle case where target is parallel/anti-parallel to original up
    if (glm::length(targetDir - originalUp) < 1e-6f) {
        return glm::mat4(1.0f); // Identity (no rotation needed)
    }
    if (glm::length(targetDir + originalUp) < 1e-6f) {
        return glm::rotate(glm::mat4(1.0f), glm::radians(180.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    }

    // Compute rotation axis and angle
    const glm::vec3 axis = glm::normalize(glm::cross(originalUp, targetDir));
    const float angle = glm::acos(glm::dot(originalUp, targetDir));
    
    return glm::rotate(glm::mat4(1.0f), angle, axis);
}

std::vector<Vertex> createSphereVertices(
    const Vector& origin_,
    const Vector& upDirection,
    float radius,
    float openingAngle,
    int segments,
    const glm::vec3& color
) {
    std::vector<Vertex> vertices;
    glm::vec3 origin{origin_.x, origin_.y, origin_.z};
    const glm::mat4 rotation = orientSphere(upDirection);

    int sectorCount = segments;
    int stackCount = segments;

    for (int i = 0; i < stackCount+1; ++i) {
        float stackAngle = M_PI_2 - i * openingAngle / stackCount; // from pi/2 to pi/2 - openingAngle
        float xy = radius * cos(stackAngle);            // r * cos(u)
        float z = radius * sin(stackAngle);             // r * sin(u)

        for (int j = 0; j < sectorCount; ++j) {
            float sectorAngle = j * M_PI * 2 / sectorCount; // from 0 to 2pi

            float x = xy * cos(sectorAngle);
            float y = xy * sin(sectorAngle);

            glm::vec3 pos = glm::vec3(x, y, z);
            pos = glm::vec3(rotation * glm::vec4(pos, 1.0f));
            glm::vec3 normal = glm::vec3(rotation * glm::vec4(glm::normalize(pos), 0.0f));

            vertices.push_back({glm::vec3(pos+origin), normal, color, 0.7f});
        }
    }
    return vertices;
}


std::vector<unsigned int> createSphereIndices(int segments, unsigned int base) {
    std::vector<unsigned int> indices;
    int stackCount = segments;
    int sectorCount = segments;
    for (unsigned int i = 0; i < stackCount; i++) {
        unsigned int k1 = i * sectorCount + base;     // beginning of current stack
        unsigned int k2 = k1 + sectorCount;      // beginning of next stack

        for (unsigned int j = 0; j < sectorCount; j++, k1++, k2++) {
            indices.insert(indices.end(), {k1, k2, k1+1});
            if (j != (sectorCount-1)) {
                indices.insert(indices.end(), {k1+1, k2, k2+1});
            } else {
                indices.insert(indices.end(), {k1-sectorCount+1, k1, k1+1});
            }
        }
    }
    return indices;
}

std::vector<Vertex> createParabolaVertices(
    const Vector& origin_,
    const Vector& height,
    float curvature,
    int segments,
    const glm::vec3& color
) {
    glm::vec3 origin{origin_.x, origin_.y, origin_.z};
    const glm::mat4 rotation = orientSphere(height);
    // create vertices in local coordinates, but push back after translation into world coordinates
    std::vector<Vertex> vertices;
    int sectorCount = segments;
    int stackCount = segments;
    for (int i = 0; i < stackCount+1; ++i) {
        float z = float(i) / float(stackCount) * height.magnitude();
        float radius = sqrt(z/curvature);
        for (int j = 0; j < sectorCount; ++j) {
            float sectorAngle = j * M_PI * 2 / sectorCount; // from 0 to 2pi

            float x = radius * cos(sectorAngle);
            float y = radius * sin(sectorAngle);

            glm::vec3 pos = glm::vec3(x, y, z);
            pos = glm::vec3(rotation * glm::vec4(pos, 1.0f));
            glm::vec3 normal = glm::vec3(rotation * glm::vec4(glm::normalize(pos), 0.0f));

            vertices.push_back({glm::vec3(pos+origin), normal, color, 0.7f});
        }
    }
    return vertices;
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
    float speed = 0.5f;
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

glm::mat4 perspectiveProjection = glm::perspective(
    glm::radians(45.0f), 
    (float)width / (float)height, 
    0.01f, 
    1000.0f
);
glm::mat4 orthographicProjection = glm::ortho(-2.0f, +2.0f, -1.5f, +1.5f, 0.1f, 100.0f);
glm::mat4 projection = perspectiveProjection;

// Keyboard callback
void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        switch (key) {
            case GLFW_KEY_W: camera.position += camera.front * camera.speed; break;
            case GLFW_KEY_S: camera.position -= camera.front * camera.speed; break;
            case GLFW_KEY_A: camera.position -= camera.right * camera.speed; break;
            case GLFW_KEY_D: camera.position += camera.right * camera.speed; break;
            case GLFW_KEY_O: projection = orthographicProjection; break;
            case GLFW_KEY_P: projection = perspectiveProjection; break;
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


glm::vec3 wavelengthToRGB(double wavelength) {
    float r, g, b;
    // No clamping as to not change input 
    // // Only consider visible spectrum (380-780 nm)
    // wavelength = std::clamp(wavelength, 380.0, 780.0);
    
    // Calculate color based on wavelength
    if (wavelength >= 380 && wavelength < 440) {
        r = -(wavelength - 440) / (440 - 380);
        g = 0.0;
        b = 1.0;
    } 
    else if (wavelength >= 440 && wavelength < 490) {
        r = 0.0;
        g = (wavelength - 440) / (490 - 440);
        b = 1.0;
    } 
    else if (wavelength >= 490 && wavelength < 510) {
        r = 0.0;
        g = 1.0;
        b = -(wavelength - 510) / (510 - 490);
    } 
    else if (wavelength >= 510 && wavelength < 580) {
        r = (wavelength - 510) / (580 - 510);
        g = 1.0;
        b = 0.0;
    } 
    else if (wavelength >= 580 && wavelength < 645) {
        r = 1.0;
        g = -(wavelength - 645) / (645 - 580);
        b = 0.0;
    } 
    else {
        r = 1.0;
        g = 0.0;
        b = 0.0;
    }
    
    // Let the intensity fall off near the vision limits
    double factor;
    if (wavelength >= 380 && wavelength < 420) {
        factor = 0.3 + 0.7 * (wavelength - 380) / (420 - 380);
    } 
    else if (wavelength >= 420 && wavelength < 700) {
        factor = 1.0;
    } 
    else {
        factor = 0.3 + 0.7 * (780 - wavelength) / (780 - 700);
    }
    
    // Apply intensity factor
    r = std::clamp(std::pow(r * factor, 0.8), 0.0, 1.0);
    g = std::clamp(std::pow(g * factor, 0.8), 0.0, 1.0);
    b = std::clamp(std::pow(b * factor, 0.8), 0.0, 1.0);
    
    return glm::vec3(r,g,b);
}

void addRays(const std::vector<Ray>& rays, 
    std::vector<Vertex>& vertices,
    std::vector<unsigned int>& indices,
    int segments
) {
    unsigned int firstIndex = 0;
    glm::vec3 color;
    for (const Ray& ray: rays) {
        if (ray.refractiveIndex == Config::VACUUM_REFRACTIVE_INDEX) {
            color = glm::vec3(0,0,1);
        } else {
            color = glm::vec3(1,0.5,0); 
        }
        if ((ray.wavelength > 380e-9) && (ray.wavelength < 780e-9)) {
            color = wavelengthToRGB(ray.wavelength);
        }
        auto cylinderVerts = createCylinder(ray.origin, ray.end, 0.005f*ray.energyDensity, segments=segments, color=color);
        for (const Vertex& cv: cylinderVerts) {
            vertices.push_back(cv);
        }

        if (indices.size() > 0) {
            firstIndex = *std::max_element(indices.begin(), indices.end()) + 1;
        }
        for (int i = 0; i < segments; ++i) {
            unsigned int base = firstIndex + i * 2;
            indices.insert(indices.end(), {base, base + 1, base + 2});
            indices.insert(indices.end(), {base + 1, base + 3, base + 2});
        }
    }
}


void visualizeWithGLFW(GeometryLoader& geometry) {
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
    glDepthMask(GL_FALSE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); 

    glfwSwapInterval(1);


    std::vector<Vertex> vertices;
    int segments = 16;
    std::vector<unsigned int> indices;
    addRays(geometry.rays, vertices, indices, segments);

    OpticalDevice* d = geometry.devices[0].get();
    for (const auto& device : geometry.devices) {
        device->createGraphicVertices(vertices, indices);
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

    // Opacity
    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, alpha));
    glEnableVertexAttribArray(3);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    glUseProgram(program);  // Compile/link shaders first
    GLint success;
    GLchar infoLog[512];

    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(program, 512, NULL, infoLog);
        std::cerr << "Shader Program linking failed:\n" << infoLog << std::endl;
    }
    glBindVertexArray(VAO);
    // glDrawArrays(GL_LINES, 0, vertices.size());  // 2 vertices per ray
    glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);

 
    while (!glfwWindowShouldClose(window))
    {
        glfwGetWindowSize(window, &width, &height);
 
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

        glm::mat4 model = glm::mat4(1.0f);
        glm::mat4 view = camera.getViewMatrix();

        glm::mat4 mvp = projection * view * glm::mat4(1.0f);
        glUseProgram(program);
        // glUniformMatrix4fv(glGetUniformLocation(program, "MVP"), 1, GL_FALSE, glm::value_ptr(mvp));
        glUniformMatrix4fv(glGetUniformLocation(program, "model"), 1, GL_FALSE, glm::value_ptr(model));
        glUniformMatrix4fv(glGetUniformLocation(program, "view"), 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(glGetUniformLocation(program, "projection"), 1, GL_FALSE, glm::value_ptr(projection));


        glBindVertexArray(VAO);
        // glDrawArrays(GL_LINES, 0, vertices.size());
        // Before rendering:
        glDepthMask(GL_FALSE);  // Disable depth writing
        glEnable(GL_BLEND);

        glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
        // glPointSize(10);
        // glDrawArrays(GL_POINTS, 0, vertices.size());

        glDepthMask(GL_TRUE);  // Re-enable depth writing
        glDisable(GL_BLEND);


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