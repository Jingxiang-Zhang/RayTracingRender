#pragma once
#ifndef _RAY_TRACING_H_
#define _RAY_TRACING_H_
#include "../model/scene.h"
#include "../in_out.h"
#include "BVH.h"

// BVH tree relevant variable
enum class RenderModeRT {
	PhoneShading,
	BRDF
};
extern RenderModeRT renderMode;

extern objectIndex objIndex;
extern SAHHybrid::LinearBVHMemory BVHTree;
extern double attenuation_diffusion;
extern double attenuation_reflection;
extern double BRDF_light_decrease;

void init_BVH();

struct pickerSetting {
	unsigned int maximum_reflection;
	unsigned int n_random_spread; // number of random spread ray
	unsigned int n_light_sampling; // used in BRDF when sampling of light
	pickerSetting() {
		maximum_reflection = 0;
		n_random_spread = 0;
		n_light_sampling = 0;
	}
};

struct ray_launcher_reture {
	enum class type {
		none, sphere, triangle
	}; 
	type intersectionType;
	union {
		Sphere* sphere; // if intersect with sphere, using this pointer
		Triangle* triangle; // if intersect with triangle, using this pointer
	};
	double position[3]; // intersection position
};

extern pickerSetting global_picker_setting;

extern double attenuation_coef_t, attenuation_coef_a, attenuation_coef_b, attenuation_coef_c;

enum class ray_launcher_type {
	naive, // triversal throuth all the object
	BVH
};

// given the ray position and the direction, launch the ray into the scene
// return intersection successfully or not
// output are the intersection position, color, and surface normal
bool ray_launcher(IN Ray& input_ray,  OUT ray_launcher_reture& ret, 
	IN ray_launcher_type type = ray_launcher_type::BVH);

bool block_test(IN double start_position[3], IN double destination[3]);

bool naive_ray_launcher(IN Ray& input_ray, OUT ray_launcher_reture& ret);

bool BVH_ray_launcher(IN Ray& input_ray, OUT ray_launcher_reture& ret);

void result_analyse(IN ray_launcher_reture& ret, OUT IntersectionInfo& info);

void pixel_picker(IN Ray& input_ray, OUT double light[3]);

void pixel_picker_phong(IN Ray& input_ray, OUT double light[3], OUT IntersectionInfo& info,
	IN int depth = 0);

void pixel_picker_BRDF(IN Ray& input_ray, OUT double light[3], OUT IntersectionInfo& info,
	IN int depth = 0);

double BRDF_pdf(IN double* point, IN double* light, IN double* normal_light);

double BRDF(IN IntersectionInfo& info, IN double* w_i, IN double* w_o, IN int channel);

inline double BRDF_ChiPlus(IN double x);

inline double BRDF_D(IN double* m, IN IntersectionInfo& info);

inline double BRDF_G1(IN double* v, IN double* m, IN IntersectionInfo& info);

inline double BRDF_G(IN double* wi, IN double* wo, IN double* h, IN IntersectionInfo& info);

inline double BRDF_F(IN double* wo, IN double* h, IN IntersectionInfo& info, IN int channel);

inline double BRDF_Fd(IN double* wi, IN double* wo, IN IntersectionInfo& info);

inline double BRDF_Fs(IN double* wi, IN double* wo, IN IntersectionInfo& info, IN int channel);

void Phong_shading(IN Ray& input_ray, IN IntersectionInfo& info, OUT double color[3]);

void BRDF_shading(IN Ray& input_ray, IN IntersectionInfo& info, OUT double color[3]);

inline double attenuation_coef(IN double P1[3], IN double P2[3]);
#endif
