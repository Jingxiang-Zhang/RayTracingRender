#pragma once
#ifndef _PLOT_H_
#define _PLOT_H_

#include <windows.h>
#include <GL/gl.h>
#include <GL/glut.h>
#include "../model/scene.h"

//you may want to make these smaller for debugging purposes
#define WIDTH 1200
#define HEIGHT 800 
extern unsigned char buffer[HEIGHT][WIDTH][3];

//different display modes
#define MODE_DISPLAY 1
#define MODE_JPEG 2
extern int mode;
//the field of view of the camera
#define fov 60.0
// maximum object size can take 80% of the screen
#define screen_factor 0.8
// view direction
extern double view_direction[3];
extern double step, start_x, start_y;
extern char* img_savepath;

enum class SuperSampling {
	X1, X4, X9, X16
};
extern SuperSampling AntiAliasing;

struct color {
	unsigned char r;
	unsigned char g;
	unsigned char b;
};

inline void SuperSamplingRayPicker(Ray& input_ray, double light[3]);

inline void plot_pixel(int x, int y, unsigned char r, unsigned char g, unsigned char b);

void display();

void init_gl();
void plot();
void init_window();
void init_viewPosition();
void init_step();

void idle();

//MODIFY THIS FUNCTION
void draw_scene();

#endif
