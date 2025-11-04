#include "hexscript.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <GLFW/glfw3.h>
#include <string.h>

#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 800
//max radius
#define MAX_SIZE 1500.0f  
//max dot size
#define DOT_SIZE 2.0f  

// Color map for intensity 
void getColor(uint8_t intensity, float* r, float* g, float* b) {
    float t = intensity / 255.0f;
    
    // Simple "hot" colormap: black -> red -> yellow -> white
    if (t < 0.33f) {
        *r = t * 3.0f;
        *g = 0.0f;
        *b = 0.0f;
    } else if (t < 0.66f) {
        *r = 1.0f;
        *g = (t - 0.33f) * 3.0f;
        *b = 0.0f;
    } else {
        *r = 1.0f;
        *g = 1.0f;
        *b = (t - 0.66f) * 3.0f;
    }
}

void renderScan(LidarScan* scan) {
    glClear(GL_COLOR_BUFFER_BIT);
    glPointSize(DOT_SIZE);
    
    glBegin(GL_POINTS);
    
    for (int i = 0; i < scan->count; i++) {
        LidarPoint* pt = &scan->points[i];
        
        //convert polar to Cartesian coordinates
        float distance_normalized = pt->distance / MAX_SIZE;
        float x = distance_normalized * cosf(pt->angle);
        float y = distance_normalized * sinf(pt->angle);
        
        //get color based on intensity
        float r, g, b;
        getColor(pt->intensity, &r, &g, &b);
        glColor3f(r, g, b);
        
        glVertex2f(x, y);
    }
    
    glEnd();
    
    //draw circular grid for reference
    glColor3f(0.2f, 0.2f, 0.2f);
    glBegin(GL_LINE_LOOP);
    for (int i = 0; i < 360; i++) {
        float angle = i * M_PI / 180.0f;
        float x = 0.5f * cosf(angle);
        float y = 0.5f * sinf(angle);
        glVertex2f(x, y);
    }
    glEnd();
    
    glBegin(GL_LINE_LOOP);
    for (int i = 0; i < 360; i++) {
        float angle = i * M_PI / 180.0f;
        float x = cosf(angle);
        float y = sinf(angle);
        glVertex2f(x, y);
    }
    glEnd();
}

void errorCallback(int error, const char* description) {
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    }
}

int main(int argc, char** argv) {
    // Default port: use macOS USB serial device when building/running on macOS,
    // otherwise use the common Linux device name. Users can still override via
    // the first command-line argument.
#ifdef __APPLE__
    const char* default_port = "/dev/tty.usbserial-0001";
#else
    const char* default_port = "/dev/ttyUSB0";
#endif
    const char* port = default_port;
    int baudrate = 230400;
    
    // Allow command-line port override
    if (argc > 1) {
        port = argv[1];
    }
    
    // Initialize LIDAR
    if (initLidar(port, baudrate) != 0) {
        fprintf(stderr, "Failed to initialize LIDAR\n");
        return 1;
    }
    
    // Initialize GLFW
    glfwSetErrorCallback(errorCallback);
    
    if (!glfwInit()) {
        fprintf(stderr, "Failed to initialize GLFW\n");
        closeLidar();
        return 1;
    }
    
    // Create window
    GLFWwindow* window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, 
                                          "LIDAR Visualization", NULL, NULL);
    if (!window) {
        fprintf(stderr, "Failed to create GLFW window\n");
        glfwTerminate();
        closeLidar();
        return 1;
    }
    
    glfwMakeContextCurrent(window);
    glfwSetKeyCallback(window, keyCallback);
    glfwSwapInterval(1); // Enable vsync (60 FPS cap)
    
    // Set up OpenGL
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glEnable(GL_POINT_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    // Main loop
    LidarScan scan;
    int frame_count = 0;
    double last_time = glfwGetTime();
    
    printf("Starting LIDAR visualization. Press ESC to exit.\n");
    
    while (!glfwWindowShouldClose(window)) {
        // Get fresh scan data
        if (getScan(&scan) != 0) {
            fprintf(stderr, "Failed to get scan data\n");
            break;
        }
        
        // Render
        renderScan(&scan);
        
        // Swap buffers and poll events
        glfwSwapBuffers(window);
        glfwPollEvents();
        
        // FPS counter
        frame_count++;
        double current_time = glfwGetTime();
        if (current_time - last_time >= 1.0) {
            printf("FPS: %d | Points: %d\n", frame_count, scan.count);
            frame_count = 0;
            last_time = current_time;
        }
    }
    
    // Cleanup
    glfwDestroyWindow(window);
    glfwTerminate();
    closeLidar();
    
    printf("Program ended.\n");
    return 0;
}