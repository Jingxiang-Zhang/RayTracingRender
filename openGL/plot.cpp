#include "plot.h"
#include "../model/load_data.h"
#include <stdio.h>
#include <iostream>
#include "../algorithm/rayTracing.h"
#include "../algorithm/parallel.h"
#include <mutex>


unsigned char buffer[HEIGHT][WIDTH][3];

char* img_savepath = nullptr;

double view_direction[3] = {0, 0, -1};

double step, start_x, start_y;

int mode = MODE_DISPLAY;

objectIndex objIndex;
SAHHybrid::HybridBVHNode* BVHroot;

SuperSampling AntiAliasing = SuperSampling::X1;

inline void plot_pixel_display(int x, int y, unsigned char r, unsigned char g, unsigned char b)
{
    glColor3f(((float)r) / 255.0f, ((float)g) / 255.0f, ((float)b) / 255.0f);
    glVertex2i(x, y);
}

inline void plot_pixel_jpeg(int x, int y, unsigned char r, unsigned char g, unsigned char b)
{
    buffer[y][x][0] = r;
    buffer[y][x][1] = g;
    buffer[y][x][2] = b;
}

void plot_pixel(int x, int y, unsigned char r, unsigned char g, unsigned char b)
{
    plot_pixel_display(x, y, r, g, b);
    if (mode == MODE_JPEG)
        plot_pixel_jpeg(x, y, r, g, b);
}

void display() {};

void init_window()
{
    glutInitDisplayMode(GLUT_RGBA | GLUT_SINGLE);
    glutInitWindowPosition(0, 0);
    glutInitWindowSize(WIDTH, HEIGHT);
    glutCreateWindow("Ray Tracer");
}

void init_viewPosition() {
    glMatrixMode(GL_PROJECTION);
    glOrtho(0, WIDTH, 0, HEIGHT, 1, -1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT);
}

void SuperSamplingRayPicker(Ray& input_ray, double light[3]) {
    switch (AntiAliasing) {
    case SuperSampling::X1:
        pixel_picker(input_ray, light);
        break;
    case SuperSampling::X4:
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                double light_temp[3] = { 0, 0, 0 };
                Ray newRay(input_ray);
                newRay.ray_source[0] += step * (i * 0.5 - 0.25);
                newRay.ray_source[1] += step * (j * 0.5 - 0.25);
                pixel_picker(newRay, light_temp);
                light[0] += light_temp[0];
                light[1] += light_temp[1];
                light[2] += light_temp[2];
            }
        }
        light[0] /= 4; light[1] /= 4; light[2] /= 4;
        break;
    case SuperSampling::X9:
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                double light_temp[3] = { 0, 0, 0 };
                Ray newRay(input_ray);
                newRay.ray_source[0] += step * (i * 0.33333333 - 0.33333333);
                newRay.ray_source[1] += step * (j * 0.33333333 - 0.33333333);
                pixel_picker(newRay, light_temp);
                light[0] += light_temp[0];
                light[1] += light_temp[1];
                light[2] += light_temp[2];
            }
        }
        light[0] /= 9; light[1] /= 9; light[2] /= 9;
        break;
    case SuperSampling::X16:
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                double light_temp[3] = { 0, 0, 0 };
                Ray newRay(input_ray);
                newRay.ray_source[0] += step * (i * 0.25 - 0.375);
                newRay.ray_source[1] += step * (j * 0.25 - 0.375);
                pixel_picker(newRay, light_temp);
                light[0] += light_temp[0];
                light[1] += light_temp[1];
                light[2] += light_temp[2];
            }
        }
        light[0] /= 16; light[1] /= 16; light[2] /= 16;
        break;
    default:
        pixel_picker(input_ray, light);
    }
}

void draw_scene()
{
    int length = std::thread::hardware_concurrency() - 1;
    length *= 3;
    // length = WIDTH;
    for (unsigned int x = 0; x < WIDTH; x += length) {
        std::vector<std::vector<color>> column_color(length,
            std::vector<color>(HEIGHT));
        parallelRun([&](int id) {
            for (unsigned int y = 0; y < HEIGHT; y++) {
                double light[3] = { 0, 0, 0 };
                Ray input_ray(start_x + step * (x + id), start_y + step * y, 1000,
                    view_direction[0], view_direction[1], view_direction[2]);
                SuperSamplingRayPicker(input_ray, light);
                light[0] = light[0] * 255;
                light[1] = light[1] * 255;
                light[2] = light[2] * 255;
                for (int i = 0; i < 3; i++) {
                    if (light[i] > 255)
                        light[i] = 255;
                    else
                        light[i] = int(light[i]);
                }
                unsigned char r, g, b;
                r = (unsigned char)light[0];
                g = (unsigned char)light[1];
                b = (unsigned char)light[2];
                column_color[id][y] = { r, g, b };
            }
        }, length);
        glPointSize(2.0);
        glBegin(GL_POINTS);
        for (unsigned int i = 0; i < length; i++) {
            for (unsigned int y = 0; y < HEIGHT; y++) {
                plot_pixel((x + i), y, column_color[i][y].r, 
                    column_color[i][y].g, column_color[i][y].b);
            }
        }
        glEnd();
        glFlush();
    }
    //for (unsigned int x = 0; x < WIDTH; x++) {
    //    glPointSize(2.0);
    //    glBegin(GL_POINTS);
    //    for (unsigned int y = 0; y < HEIGHT; y++) {
    //        unsigned char r, g, b;
    //        Ray input_ray(start_x + step * x, start_y + step * y, 1000,
    //            view_direction[0], view_direction[1], view_direction[2]);
    //        pixel_picker(input_ray, r, g, b);
    //        plot_pixel(x, y, r, g, b);
    //    }
    //    glEnd();
    //    glFlush();
    //}
}

void idle()
{
    //hack to make it only draw once
    static int once = 0;
    if (!once)
    {
        draw_scene();
        if (mode == MODE_JPEG)
            save_jpg(img_savepath);
    }
    once = 1;
}

void init_gl() {
    int argc = 0;
    char** argv = nullptr;
    glutInit(&argc, argv);

    init_step();
    init_window();
    init_viewPosition();
    init_BVH();
    glutDisplayFunc(display);
    glutIdleFunc(idle);
}

void plot() {
    glutMainLoop();
}

void init_step() {
    double ratio_scene = (sceneStatistic.max_x - sceneStatistic.min_x) /
        (sceneStatistic.max_y - sceneStatistic.min_y);
    double ratio_screen = WIDTH / HEIGHT;
    if (ratio_scene > ratio_screen)
        step = (sceneStatistic.max_x - sceneStatistic.min_x) / (WIDTH * screen_factor);
    else
        step = (sceneStatistic.max_y - sceneStatistic.min_y) / (HEIGHT * screen_factor);
    start_x = sceneStatistic.min_x - (WIDTH * (1 - screen_factor) / 2) * step;
    start_y = sceneStatistic.min_y - (HEIGHT * (1 - screen_factor) / 2) * step;
}