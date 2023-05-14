#pragma once
#ifndef _GEOMETRY_H_
#define _GEOMETRY_H_
#include "../in_out.h"
#include "scene.h"
#include "../algorithm/BVH.h"
#define GEOAPI extern inline
#define PI 3.1415926

GEOAPI void normalize(INOUT double direction[3]);

GEOAPI void random_sample(IN double normal[3], IN int number, OUT double direction[][3]);

GEOAPI void random_sample_light(IN int number, OUT double position[][3], 
	OUT double normal[][3], OUT double color[][3], OUT double light_area[]);

GEOAPI void cross_product(IN double P1[3], IN double P2[3], OUT double cross[3]);

GEOAPI double dot_product(IN double P1[3], IN double P2[3]);

GEOAPI double vector_length(IN double vec[3]);

GEOAPI double distance(IN double P1[3], IN double P2[3]);

GEOAPI void reflect(IN double input[3], IN double normal[3], OUT double output[3]);

GEOAPI bool ray_shpere_intersection(IN Ray& input_ray, IN Sphere& sphere, OUT double& t,
	OUT double position[3]);

GEOAPI bool ray_triangle_intersection(IN Ray& input_ray, IN Triangle& triangle, OUT double& t,
	OUT double position[3]);

GEOAPI bool ray_box_intersection(IN double* raySource, IN double* rayDirection,
	IN SAHHybrid::BoundingBox& box);

GEOAPI bool ray_object_intersection(IN Ray& input_ray, IN Object& object, OUT double& t,
	OUT double position[3]);

GEOAPI void sphere_normal(IN Sphere sphere, IN double intersection_pos[3], OUT double normal[3]);

// calculate alpha, beta, and gamma
GEOAPI void triangle_interpolation(IN Triangle& triangle, IN double position[3],
	OUT Vertex& vec);

GEOAPI double triangle_area(IN double p1[3], IN double p2[3], IN double p3[3]);

#endif
