#pragma once

#ifndef _SCENE_H_
#define _SCENE_H_

#define MAX_TRIANGLES 20000
#define MAX_SPHERES 100
#define MAX_LIGHTS 100
#include <vector>

struct Vertex {
    double position[3];
    double color_diffuse[3];
    double color_specular[3];
    double normal[3];
    double shininess;
    double rou;
    double met;
};

struct Triangle {
    Vertex v[3];
};

struct Sphere {
    double position[3];
    double color_diffuse[3];
    double color_specular[3];
    double shininess;
    double radius;
    double rou;
    double met;
};

struct IntersectionInfo {
    double shininess;
    double position[3];
    double normal[3];
    double color_diffuse[3];
    double color_specular[3];
    double rou;
    double met;
};

struct Light {
    double position[3];
    double color[3];
    double p0[3];
    double p1[3];
    double p2[3];
    double p3[3];
    double normal[3];
    double area;
};

extern double back_ground_color[3];

extern double La[3];
extern double Ld[3];
extern double Ls[3];
extern double ka[3];

struct SceneStatistic {
    double min_x, max_x;
    double min_y, max_y;
    double min_z, max_z;
};

extern Triangle triangles[MAX_TRIANGLES];
extern Sphere spheres[MAX_SPHERES];
extern Light lights[MAX_LIGHTS];
extern double ambient_light[3];
extern double f0[3];

extern int num_triangles;
extern int num_spheres;
extern int num_lights;
extern SceneStatistic sceneStatistic;

struct Object {
    enum class objType {
        triangle,
        sphere
    };
    objType type;
    union pointerType{
        Triangle* triangle;
        Sphere* sphere;
    };
    pointerType pointer;
};

typedef std::vector<unsigned int> objectIndex;

extern std::vector<Object> obj_list;

struct Ray {
    double ray_source[3];
    double ray_direction[3];
    Ray(double ray_source_x, double ray_source_y, double ray_source_z,
        double ray_direction_x, double ray_direction_y, double ray_direction_z) {
        ray_source[0] = ray_source_x; 
        ray_source[1] = ray_source_y; 
        ray_source[2] = ray_source_z;
        ray_direction[0] = ray_direction_x;
        ray_direction[1] = ray_direction_y;
        ray_direction[2] = ray_direction_z;
    }
    Ray() {};
};

#endif

