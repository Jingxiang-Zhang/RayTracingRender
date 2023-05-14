#include "scene.h"

Triangle triangles[MAX_TRIANGLES];
Sphere spheres[MAX_SPHERES];
Light lights[MAX_LIGHTS];
double ambient_light[3];
double f0[3];
std::vector<Object> obj_list;

int num_triangles = 0;
int num_spheres = 0;
int num_lights = 0;
SceneStatistic sceneStatistic;
double La[3] = { 0.2, 0.2, 0.2 };
double Ld[3] = { 0.6, 0.6, 0.6 };
double Ls[3] = { 0.8, 0.8, 0.8 };
double ka[3] = { 1.0, 1.0, 1.0 };
double back_ground_color[3] = { 0, 0, 0 };
