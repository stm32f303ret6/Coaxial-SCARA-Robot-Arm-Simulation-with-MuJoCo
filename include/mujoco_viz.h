#ifndef MUJOCOVIZ_H
#define MUJOCOVIZ_H

#include <GLFW/glfw3.h>
#include <mujoco.h>
#include <stdio.h>

mjModel *m = NULL; // MuJoCo model
mjData *d = NULL;  // MuJoCo data
mjvCamera cam;     // abstract camera
mjvOption opt;     // visualization options
mjvScene scn;      // abstract scene
mjrContext con;    // custom GPU context


// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;

float mj_vx = 0, mj_vy = 0, mj_w = 0;
char mj_take_pointcloud = 0;
float height=0.02, gripper=0.0;

// keyboard callback
void keyboard(GLFWwindow *window, int key, int scancode, int act, int mods)
{
    if (key == GLFW_KEY_W && act == GLFW_PRESS)
        height += 0.01;
    else if (key == GLFW_KEY_S && act == GLFW_PRESS)
        height -= 0.01;
    else if (key == GLFW_KEY_A && act == GLFW_PRESS)
        gripper = 10;
    else if (key == GLFW_KEY_D && act == GLFW_PRESS)
        gripper = -10;
    else if (key == GLFW_KEY_R && act == GLFW_PRESS)
        mj_w += 0.1;
    else if (key == GLFW_KEY_F && act == GLFW_PRESS)
        mj_w += 0.1;
    else if (key == GLFW_KEY_Q && act == GLFW_PRESS)
    {
        mj_vx = 0.0;
        mj_vy = 0.0;
        mj_w = 0.0;
    }
    else if (key == GLFW_KEY_T && act == GLFW_PRESS)
        mj_take_pointcloud = 1;
}

// mouse button callback
void mouse_button(GLFWwindow *window, int button, int act, int mods)
{
    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}

// mouse move callback
void mouse_move(GLFWwindow *window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if (!button_left && !button_middle && !button_right)
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if (button_right)
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left)
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

// scroll callback
void scroll(GLFWwindow *window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, 0.05 * yoffset, &scn, &cam);
}

void mujoco_loop()
{
    if (!glfwInit())
        mju_error("Could not initialize GLFW");
    // glfwWindowHint(GLFW_SAMPLES, 0);
    GLFWwindow *window = glfwCreateWindow(1244, 700, "Demo", NULL, NULL);
    // glEnable(GL_MULTISAMPLE);
    glEnable(GL_DEPTH_TEST);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);
    cam.distance = 0.4;

    while (!glfwWindowShouldClose(window))
    {
        mjtNum simstart = d->time;
        while (d->time - simstart < 1.0 / 60.0)
        {
            mj_step(m, d);
        }

        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        //cam.lookat[0] = d->qpos[0];
        //cam.lookat[1] = d->qpos[1];
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    mj_deleteData(d);
    mj_deleteModel(m);
}

#endif // MUJOCOVIZ_H
