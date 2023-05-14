#include "rayTracing.h"
#include "../model/geometry.h"
#include <string.h>
#include <iostream>
#include <algorithm>

pickerSetting global_picker_setting;
double attenuation_diffusion = 0.2;
double attenuation_reflection = 0.1;
double attenuation_coef_t = 100.0;
double attenuation_coef_a = 1.0;
double attenuation_coef_b = 0;
double attenuation_coef_c = 100.0;
double BRDF_light_decrease;
RenderModeRT renderMode;
SAHHybrid::LinearBVHMemory BVHTree;

bool ray_launcher(IN Ray& input_ray, OUT ray_launcher_reture& ret, IN ray_launcher_type type) {
	if (type == ray_launcher_type::naive) {
		return naive_ray_launcher(input_ray, ret);
	}
	else if(type == ray_launcher_type::BVH) {
		if (BVHTree.size() == 0)
			return false;
		return BVH_ray_launcher(input_ray, ret);
	}
}

bool BVH_ray_launcher(IN Ray& input_ray, OUT ray_launcher_reture& ret) {
	int toVisitOffset = 0, currentNodeIndex = 0;
	int nodesToVisit[64];
	double minimal_time = 1000000;  // nearest point
	bool success = false;
	while (true) {
		SAHHybrid::LinearBVHNode* node = &BVHTree[currentNodeIndex];
		//std::cout << node->bounds.x_min <<" "<< node->bounds.x_max <<" " 
		//	<< node->bounds.y_min << " " << node->bounds.y_max << " " 
		//	<< node->bounds.z_min << " " << node->bounds.z_max << " " << std::endl;
		if (ray_box_intersection(input_ray.ray_source, input_ray.ray_direction, 
			node->bounds)) {
			if (node->nPrimitives > 0) {
				for (int i = node->primitivesOffset;
					i < node->primitivesOffset + node->nPrimitives; i++) {
					Object& obj = obj_list[objIndex[i]];
					double time;
					double position[3];
					bool flag = ray_object_intersection(input_ray, obj, time, position);
					if (flag && minimal_time > time) {
						success = true;
						if (obj.type == Object::objType::sphere) {
							ret.intersectionType = ray_launcher_reture::type::sphere;
							ret.sphere = obj.pointer.sphere;
						}
						else if (obj.type == Object::objType::triangle) {
							ret.intersectionType = ray_launcher_reture::type::triangle;
							ret.triangle = obj.pointer.triangle;
						}
						minimal_time = time;
						memcpy(ret.position, position, sizeof(double) * 3);
					}
				}
				if (toVisitOffset == 0)
					break;
				currentNodeIndex = nodesToVisit[--toVisitOffset];
			}
			else {
				currentNodeIndex = currentNodeIndex + 1;
				nodesToVisit[toVisitOffset++] = node->secondChildOffset;
			}
		}
		else {
			if (toVisitOffset == 0)
				break;
			toVisitOffset--;
			currentNodeIndex = nodesToVisit[toVisitOffset];
		}
	}
	return success;
}

bool naive_ray_launcher(IN Ray& input_ray, OUT ray_launcher_reture& ret) {
	double minimum_time = 10000000.0;
	bool success = false;
	// test with all objects
	for (auto obj : obj_list) {
		double time;
		double position[3];
		bool flag = ray_object_intersection(input_ray, obj, time, position);
		if (flag && time < minimum_time) {
			success = true;
			if (obj.type == Object::objType::sphere) {
				ret.intersectionType = ray_launcher_reture::type::sphere;
				ret.sphere = obj.pointer.sphere;
			}
			else if (obj.type == Object::objType::triangle) {
				ret.intersectionType = ray_launcher_reture::type::triangle;
				ret.triangle = obj.pointer.triangle;
			}
			minimum_time = time;
			memcpy(ret.position, position, sizeof(double) * 3);
		}
	}
	return success;
}

void result_analyse(IN ray_launcher_reture& ret, OUT IntersectionInfo& info) {
	if (ret.intersectionType == ray_launcher_reture::type::sphere) {
		info.shininess = ret.sphere->shininess;
		info.rou = ret.sphere->rou;
		info.met = ret.sphere->met;
		memcpy(info.position, ret.position, sizeof(double) * 3);
		sphere_normal(*ret.sphere, ret.position, info.normal);
		memcpy(info.color_diffuse, ret.sphere->color_diffuse, sizeof(double) * 3);
		memcpy(info.color_specular, ret.sphere->color_specular, sizeof(double) * 3);
	}
	else if (ret.intersectionType == ray_launcher_reture::type::triangle) {
		Vertex interpolation;
		triangle_interpolation(*ret.triangle, ret.position, interpolation);
		info.shininess = interpolation.shininess;
		info.rou = interpolation.rou;
		info.met = interpolation.met;
		memcpy(info.position, ret.position, sizeof(double) * 3);
		memcpy(info.normal, interpolation.normal, sizeof(double) * 3);
		memcpy(info.color_diffuse, interpolation.color_diffuse, sizeof(double) * 3);
		memcpy(info.color_specular, interpolation.color_specular, sizeof(double) * 3);
	}
}

bool block_test(IN double start_position[3], IN double destination[3]) {
	Ray shadow_ray(start_position[0], start_position[1], start_position[2],
		destination[0] - start_position[0], destination[1] - start_position[1],
		destination[2] - start_position[2]);
	normalize(shadow_ray.ray_direction);
	ray_launcher_reture launcher_result;
	bool flag = ray_launcher(shadow_ray, launcher_result);
	if (flag) { // if the ray intersect with some object, which is not what we want
		// test the distance
		double intersection_to_block_length =
			distance(start_position, launcher_result.position);
		double intersection_to_light_length =
			distance(start_position, destination);
		if (intersection_to_block_length < intersection_to_light_length) {
			return false;
		}
	}
	return true;
}

double attenuation_coef(IN double P1[3], IN double P2[3]) {
	double d = distance(P1, P2);
	return attenuation_coef_t / (attenuation_coef_a * d * d + attenuation_coef_b * d +
		attenuation_coef_c);
}

void Phong_shading(IN Ray& input_ray, IN IntersectionInfo& info, OUT double color[3]) {
	double ambient[3] = { La[0] * ka[0], La[1] * ka[1], La[2] * ka[2] };
	double diffuse[3] = { 0.0, 0.0, 0.0 };
	double specular[3] = { 0.0, 0.0, 0.0 };
	for (int i = 0; i < num_lights; i++) {
		Light& light = lights[i];
		// launch shadow ray
		// construct a ray from the intersection position to the ray source
		double* start_position = info.position;
		double* destination = light.position;
		bool shadow_ray_test = block_test(start_position, destination);
		if (!shadow_ray_test)
			continue;
		// calculate the attenuation
		double coef = attenuation_coef(info.position, destination);
		// calculate diffuse term
		double l_vec[3] = { destination[0] - start_position[0],
			destination[1] - start_position[1],
			destination[2] - start_position[2] };
		normalize(l_vec);
		double factor = dot_product(info.normal, l_vec);
		diffuse[0] += (factor * Ld[0] * info.color_diffuse[0]) * coef;
		diffuse[1] += (factor * Ld[1] * info.color_diffuse[1]) * coef;
		diffuse[2] += (factor * Ld[2] * info.color_diffuse[2]) * coef;
		// calculate specular term
		double r_vec[3];
		reflect(l_vec, info.normal, r_vec);
		double v_vec[3] = {
			input_ray.ray_source[0] - start_position[0],
			input_ray.ray_source[1] - start_position[1],
			input_ray.ray_source[2] - start_position[2]
		};
		normalize(v_vec);
		factor = pow(std::max(0.0, dot_product(r_vec, v_vec)), info.shininess);
		// std::cout << factor << std::endl;
		specular[0] += (factor * Ls[0] * info.color_specular[0]) * coef;
		specular[1] += (factor * Ls[1] * info.color_specular[1])* coef;
		specular[2] += (factor * Ls[2] * info.color_specular[2])* coef;
	}
	double phone_color[3] = {
		ambient[0] + diffuse[0] + specular[0],
		ambient[1] + diffuse[1] + specular[1],
		ambient[2] + diffuse[2] + specular[2]
	};
	color[0] = phone_color[0];
	color[1] = phone_color[1];
	color[2] = phone_color[2];
}

void pixel_picker(IN Ray& input_ray, OUT double light[3]) {
	IntersectionInfo info;
	switch (renderMode) {
	case RenderModeRT::PhoneShading:
		pixel_picker_phong(input_ray, light, info);
		break;
	case RenderModeRT::BRDF:
		pixel_picker_BRDF(input_ray, light, info);
		break;
	default:
		pixel_picker_phong(input_ray, light, info);
	}
}

double BRDF_pdf(IN double* point, IN double* light, IN double* normal_light) {
	double direction[3] = {
		point[0] - light[0],
		point[1] - light[1],
		point[2] - light[2]
	};
	double dist_square = vector_length(direction);
	normalize(direction);
	double coef = std::abs(dot_product(normal_light, direction));
	return dist_square / coef;
	// return dist_square / (coef * global_picker_setting.n_light_sampling);
}

double BRDF_ChiPlus(IN double x) {
	return x > 0 ? 1 : 0;
}

double BRDF_D(IN double* m, IN IntersectionInfo& info) {
	double alpha = info.rou * info.rou;
	double numerator = alpha * alpha * BRDF_ChiPlus(dot_product(m, info.normal));
	double theta_m = std::acos(dot_product(info.normal, m));
	double denominator = PI * std::pow(std::cos(theta_m), 4) * std::pow(
		alpha * alpha + std::tan(theta_m) * std::tan(theta_m), 2);
	return numerator / denominator;
}

double BRDF_G(IN double* wi, IN double* wo, IN double* h, IN IntersectionInfo& info) {
	return BRDF_G1(wi, h, info) * BRDF_G1(wo, h, info);
}

double BRDF_G1(IN double* v, IN double* m, IN IntersectionInfo& info) {
	double alpha = info.rou * info.rou;
	double coef = BRDF_ChiPlus(
		dot_product(v, m) / dot_product(v, info.normal)
	);
	double theta_v = std::acos(dot_product(info.normal, v));
	double denominator = 1 + std::sqrt(
		1 + alpha * alpha * std::tan(theta_v) * std::tan(theta_v)
	);
	return 2.0 * coef / denominator;
}

double BRDF_F(IN double* wo, IN double* h, IN IntersectionInfo& info, IN int channel) {
	return info.color_diffuse[channel]
		+ (1 - info.color_diffuse[channel]) * std::pow(1 - dot_product(wo, h), 5);
}

double BRDF_Fd(IN double* wi, IN double* wo, IN IntersectionInfo& info) {
	double h[3] = { wi[0] + wo[0], wi[1] + wo[1],  wi[2] + wo[2] };
	normalize(h);
	if (dot_product(wi, info.normal) < 0) {
		h[0] = -h[0]; h[1] = -h[1]; h[2] = -h[2];
	}
	double F_D90 = 2.0 * std::pow(dot_product(h, wi), 2) * info.rou + 0.5;
	double part_1 = (1 + (F_D90 - 1) * std::pow((1 - dot_product(wi, info.normal)), 5));
	double part_2 = (1 + (F_D90 - 1) * std::pow((1 - dot_product(wo, info.normal)), 5));
	double part_3 = (1 - info.met);
	return part_1 * part_2 * part_3 / PI;
}

double BRDF_Fs(IN double* wi, IN double* wo, IN IntersectionInfo& info, IN int channel) {
	double h[3] = { wi[0] + wo[0], wi[1] + wo[1],  wi[2] + wo[2] };
	normalize(h);
	if (dot_product(wi, info.normal) < 0) {
		h[0] = -h[0]; h[1] = -h[1]; h[2] = -h[2];
	}
	double numerator = BRDF_F(wo, h, info, channel) * BRDF_G(wi, wo, h, info) *
		BRDF_D(h, info);
	double denominator = 4 * std::abs(dot_product(wi, info.normal)) * 
		std::abs(dot_product(wo, info.normal));
	return numerator / denominator;
}

double BRDF(IN IntersectionInfo& info, IN double* wi, IN double* wo, IN int channel) {
	return (BRDF_Fs(wi, wo, info, channel) + 
		BRDF_Fd(wi, wo, info)) * info.color_diffuse[channel];
}

void BRDF_shading(IN Ray& input_ray, IN IntersectionInfo& info, OUT double color[3]) {
	int nLights = global_picker_setting.n_light_sampling;
	double* light_position_list = new double[nLights * 3];
	double* light_normal_list = new double[nLights * 3];
	double* light_color_list = new double[nLights * 3];
	double* light_area = new double[nLights];
	random_sample_light(nLights, (double(*)[3])light_position_list,
		(double(*)[3])light_normal_list, (double(*)[3])light_color_list, light_area);
	for (int i = 0; i < global_picker_setting.n_light_sampling; i++) {
		double light_position[3] = { light_position_list[i * 3],
			light_position_list[i * 3 + 1],
			light_position_list[i * 3 + 2] };
		bool flag = block_test(info.position, light_position);
		if (!flag) { // the light is blocked
			continue;
		}
		double light_normal[3] = { light_normal_list[i * 3],
			light_normal_list[i * 3 + 1],
			light_normal_list[i * 3 + 2] };
		double light_color[3] = { light_color_list[i * 3],
			light_color_list[i * 3 + 1],
			light_color_list[i * 3 + 2] };
		double pdf = BRDF_pdf(info.position, light_position, light_normal) * light_area[i];
		double w_i[3] = { light_position[0] - info.position[0],
			light_position[1] - info.position[1],
			light_position[2] - info.position[2] };
		double w_o[3] = { input_ray.ray_source[0] - info.position[0],
			input_ray.ray_source[1] - info.position[1],
			input_ray.ray_source[2] - info.position[2] };
		normalize(w_i);
		normalize(w_o);
		double brdf_0 = BRDF(info, w_i, w_o, 0);
		double brdf_1 = BRDF(info, w_i, w_o, 1);
		double brdf_2 = BRDF(info, w_i, w_o, 2);
		double dot = dot_product(w_i, info.normal);
		color[0] += light_color[0] * brdf_0 * dot / pdf;
		color[1] += light_color[1] * brdf_1 * dot / pdf;
		color[2] += light_color[2] * brdf_2 * dot / pdf;
		//color[0] += brdf_0 * dot / pdf;
		//color[1] += brdf_1 * dot / pdf;
		//color[2] += brdf_2 * dot / pdf;
	}
	delete[] light_position_list;
	delete[] light_normal_list;
	delete[] light_color_list;
	delete[] light_area;
	color[0] /= nLights * BRDF_light_decrease;
	color[1] /= nLights * BRDF_light_decrease;
	color[2] /= nLights * BRDF_light_decrease;
}

void pixel_picker_BRDF(IN Ray& input_ray, OUT double light[3], OUT IntersectionInfo& info,
	IN int depth) {
	depth++;
	ray_launcher_reture launcher_result;
	bool success = ray_launcher(input_ray, launcher_result);
	if (success) {
		double L_dir[3] = { 0.0, 0.0, 0.0 };
		result_analyse(launcher_result, info);
		BRDF_shading(input_ray, info, L_dir);
		if (depth > global_picker_setting.maximum_reflection) {
			memcpy(light, L_dir, sizeof(double) * 3);
			return;
		}
		// reflection
		double L_reflect[3] = { 0, 0, 0 };
		{
			// calculate the reflection direction
			double l_vec[3] = {
				-input_ray.ray_direction[0],
				-input_ray.ray_direction[1],
				-input_ray.ray_direction[2]
			};
			normalize(l_vec);
			double r_vec[3];
			reflect(l_vec, info.normal, r_vec);
			// sampling the color of the given direction
			Ray reflect_ray(info.position[0], info.position[1], info.position[2],
				r_vec[0], r_vec[1], r_vec[2]);
			IntersectionInfo reflect_info;
			double reflect_color[3] = { 0, 0, 0 };
			pixel_picker_BRDF(reflect_ray, reflect_color, reflect_info, depth + 1);
			// calculate the attenuation
			double coef = attenuation_coef(info.position, reflect_info.position);
			// calculate diffuse term
			double factor = dot_product(info.normal, r_vec);
			L_reflect[0] = (factor * reflect_color[0] * info.color_diffuse[0]) * coef;
			L_reflect[1] = (factor * reflect_color[1] * info.color_diffuse[1]) * coef;
			L_reflect[2] = (factor * reflect_color[2] * info.color_diffuse[2]) * coef;
		}

		//std::cout << L_diffuse[0] << " " << L_diffuse[1] << std::endl;
		light[0] = L_dir[0] + L_reflect[0] * attenuation_reflection;
		light[1] = L_dir[1] + L_reflect[1] * attenuation_reflection;
		light[2] = L_dir[2] + L_reflect[2] * attenuation_reflection;
	}
	else
		memcpy(light, back_ground_color, sizeof(double) * 3);
}

void pixel_picker_phong(IN Ray& input_ray, OUT double light[3], OUT IntersectionInfo& info,
	IN int depth) {
	depth++;
	ray_launcher_reture launcher_result;
	bool success = ray_launcher(input_ray, launcher_result);
	if (success) {
		double L_dir[3];
		result_analyse(launcher_result, info);
		Phong_shading(input_ray, info, L_dir);
		if (depth > global_picker_setting.maximum_reflection) {
			memcpy(light, L_dir, sizeof(double) * 3);
			return;
		}
		// diffusion
		double L_diffuse[3] = { 0,0,0 };
		// double L_diffuse[3];
		{
			int nRays = global_picker_setting.n_random_spread;
			double* direction = new double[nRays * 3];
			random_sample(info.normal, nRays, (double(*)[3])direction);
			for (int i = 0; i < nRays; i++) {
				double diffuse_direction[3] = {
					direction[i * 3],
					direction[i * 3 + 1],
					direction[i * 3 + 2]
				};
				Ray reflect_ray(info.position[0], info.position[1], info.position[2],
					diffuse_direction[0], diffuse_direction[1], diffuse_direction[2]);
				IntersectionInfo reflect_info;
				double diffuse_color[3] = { 0, 0, 0 };
				pixel_picker_phong(reflect_ray, diffuse_color, reflect_info, 10000);
				// calculate the attenuation
				double coef = attenuation_coef(info.position, reflect_info.position);
				// calculate diffuse term
				double factor = dot_product(info.normal, diffuse_direction);
				L_diffuse[0] += (factor * diffuse_color[0] * info.color_diffuse[0]) * coef;
				L_diffuse[1] += (factor * diffuse_color[1] * info.color_diffuse[1]) * coef;
				L_diffuse[2] += (factor * diffuse_color[2] * info.color_diffuse[2]) * coef;
				// calculate specular term
				double r_vec[3];
				reflect(diffuse_direction, info.normal, r_vec);
				double v_vec[3] = {
					-input_ray.ray_direction[0],
					-input_ray.ray_direction[1],
					-input_ray.ray_direction[2]
				};
				normalize(v_vec);
				factor = pow(std::max(0.0, dot_product(r_vec, v_vec)), info.shininess);
				L_diffuse[0] += (diffuse_color[0] * info.color_specular[0]) * coef;
				L_diffuse[1] += (diffuse_color[1] * info.color_specular[1]) * coef;
				L_diffuse[2] += (diffuse_color[2] * info.color_specular[2]) * coef;
			}
			delete[] direction;
			if (nRays != 0) {
				L_diffuse[0] = L_diffuse[0] / nRays;
				L_diffuse[1] = L_diffuse[1] / nRays;
				L_diffuse[2] = L_diffuse[2] / nRays;
			}
		}
		// reflection
		double L_reflect[3] = {0, 0, 0};
		{
			// calculate the reflection direction
			double l_vec[3] = {
				-input_ray.ray_direction[0],
				-input_ray.ray_direction[1],
				-input_ray.ray_direction[2]
			};
			normalize(l_vec);
			double r_vec[3];
			reflect(l_vec, info.normal, r_vec);
			// sampling the color of the given direction
			Ray reflect_ray(info.position[0], info.position[1], info.position[2],
				r_vec[0], r_vec[1], r_vec[2]);
			IntersectionInfo reflect_info;
			double reflect_color[3] = { 0, 0, 0 };
			pixel_picker_phong(reflect_ray, reflect_color, reflect_info, depth + 1);
			// calculate the attenuation
			double coef = attenuation_coef(info.position, reflect_info.position);
			// calculate diffuse term
			double factor = dot_product(info.normal, r_vec);
			L_reflect[0] += (factor * reflect_color[0] * info.color_diffuse[0]) * coef;
			L_reflect[1] += (factor * reflect_color[1] * info.color_diffuse[1]) * coef;
			L_reflect[2] += (factor * reflect_color[2] * info.color_diffuse[2]) * coef;
			// calculate specular term
			L_reflect[0] += (reflect_color[0] * info.color_specular[0]) * coef;
			L_reflect[1] += (reflect_color[1] * info.color_specular[1]) * coef;
			L_reflect[2] += (reflect_color[2] * info.color_specular[2]) * coef;
		}

		//std::cout << L_diffuse[0] << " " << L_diffuse[1] << std::endl;
		light[0] = L_dir[0] + L_reflect[0] * attenuation_reflection + L_diffuse[0] * attenuation_diffusion;
		light[1] = L_dir[1] + L_reflect[1] * attenuation_reflection + L_diffuse[1] * attenuation_diffusion;
		light[2] = L_dir[2] + L_reflect[2] * attenuation_reflection + L_diffuse[2] * attenuation_diffusion;

		//light[0] = L_dir[0] + L_diffuse[0] * 0.2;
		//light[1] = L_dir[1] + L_diffuse[1] * 0.2;
		//light[2] = L_dir[2] + L_diffuse[2] * 0.2;
	}
	else
		memcpy(light, back_ground_color, sizeof(double) * 3);
}

void init_BVH() {
	// assign each object with an unique index
	objIndex.reserve(obj_list.size());
	for (unsigned int i = 0; i < obj_list.size(); i++) 
		objIndex.push_back(i);
	BVHTree = SAHHybrid::BVHBuildByhybridSAH(objIndex);
}