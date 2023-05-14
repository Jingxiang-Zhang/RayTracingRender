#include "geometry.h"
#include <algorithm>
#include <iostream>
#include <ctime>
#include <random>
#include <chrono>

void cross_product(IN double P1[3], IN double P2[3], OUT double cross[3]) {
	cross[0] = P1[1] * P2[2] - P1[2] * P2[1];
	cross[1] = P1[2] * P2[0] - P1[0] * P2[2];
	cross[2] = P1[0] * P2[1] - P1[1] * P2[0];
}

double dot_product(IN double P1[3], IN double P2[3]) {
	return P1[0] * P2[0] + P1[1] * P2[1] + P1[2] * P2[2];
}

void normalize(INOUT double direction[3]) {
	double sum_ = 0.0f;
	sum_ += direction[0] * direction[0] + direction[1] * direction[1] +
		direction[2] * direction[2];
	sum_ = 1 / sqrt(sum_);
	direction[0] *= sum_;
	direction[1] *= sum_;
	direction[2] *= sum_;
}

double vector_length(IN double vec[3]) {
	return sqrt(
		vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]
	);
}

bool ray_shpere_intersection(IN Ray& input_ray, IN Sphere& sphere, OUT double& t,
	OUT double position[3]) {
	double ray_to_sphere[3] = {
		input_ray.ray_source[0] - sphere.position[0],
		input_ray.ray_source[1] - sphere.position[1],
		input_ray.ray_source[2] - sphere.position[2]
	};
	double b = 2 * (input_ray.ray_direction[0] * ray_to_sphere[0] +
		input_ray.ray_direction[1] * ray_to_sphere[1] +
		input_ray.ray_direction[2] * ray_to_sphere[2]);
	double c = ray_to_sphere[0] * ray_to_sphere[0] + ray_to_sphere[1] * ray_to_sphere[1] +
		ray_to_sphere[2] * ray_to_sphere[2] - sphere.radius * sphere.radius;
	double flag = b * b - 4 * c;
	if (flag < 0)
		return false;
	flag = sqrt(flag);
	double t1 = (-b - flag) / 2;
	double t2 = (-b + flag) / 2;
	double e = 0.000001;
	if (t1 > e)
		t = t1;
	else {
		if (t2 < e)
			return false;
		t = t2;
	}
	//t = (-b - flag) / 2;
	//if (t >= -0.00001 && t <= 0.00001) {
	//	t = (-b + flag) / 2;
	//}
	//if (t <= 0)
	//	return false;
	position[0] = input_ray.ray_source[0] + input_ray.ray_direction[0] * t;
	position[1] = input_ray.ray_source[1] + input_ray.ray_direction[1] * t;
	position[2] = input_ray.ray_source[2] + input_ray.ray_direction[2] * t;
	return true;
}

bool ray_box_intersection(IN double* raySource, IN double* rayDirection,
	IN SAHHybrid::BoundingBox& box) {
	double tmin = -100000000.0f;
	double tmax = 100000000.0f;
	double temp1 = 0.0f, temp2 = 0.0f, temp_min = 0.0f, temp_max = 0.0f;
	if (rayDirection[0] != 0) {
		temp1 = (box.x_min - raySource[0]) / rayDirection[0];
		temp2 = (box.x_max - raySource[0]) / rayDirection[0];
		temp_min = temp1 < temp2 ? temp1 : temp2;
		temp_max = temp1 > temp2 ? temp1 : temp2;
		tmin = tmin > temp_min ? tmin : temp_min;
		tmax = tmax < temp_max ? tmax : temp_max;
	}
	else if (raySource[0] < box.x_min || raySource[0] > box.x_max)
		return false;
	if (rayDirection[1] != 0) {
		temp1 = (box.y_min - raySource[1]) / rayDirection[1];
		temp2 = (box.y_max - raySource[1]) / rayDirection[1];
		temp_min = temp1 < temp2 ? temp1 : temp2;
		temp_max = temp1 > temp2 ? temp1 : temp2;
		tmin = tmin > temp_min ? tmin : temp_min;
		tmax = tmax < temp_max ? tmax : temp_max;
	}
	else if (raySource[1] < box.y_min || raySource[1] > box.y_max)
		return false;
	if (rayDirection[2] != 0) {
		temp1 = (box.z_min - raySource[2]) / rayDirection[2];
		temp2 = (box.z_max - raySource[2]) / rayDirection[2];
		temp_min = temp1 < temp2 ? temp1 : temp2;
		temp_max = temp1 > temp2 ? temp1 : temp2;
		tmin = tmin > temp_min ? tmin : temp_min;
		tmax = tmax < temp_max ? tmax : temp_max;
	}
	else if (raySource[2] < box.z_min || raySource[2] > box.z_max)
		return false;
	if (tmax >= 0 && tmax >= tmin) {
		return true;
	}

	return false;
}

bool ray_object_intersection(IN Ray& input_ray, IN Object& object, OUT double& t,
	OUT double position[3]) {
	if (object.type == Object::objType::sphere) {
		Sphere& sphere = *object.pointer.sphere;
		return ray_shpere_intersection(input_ray, sphere, t, position);
	}
	else if (object.type == Object::objType::triangle) {
		Triangle& triangle = *object.pointer.triangle;
		return ray_triangle_intersection(input_ray, triangle, t, position);
	}
}

void reflect(IN double input[3], IN double normal[3], OUT double output[3]) {
	double dot = dot_product(input, normal) * 2;
	output[0] = dot * normal[0] - input[0];
	output[1] = dot * normal[1] - input[1];
	output[2] = dot * normal[2] - input[2];
}

bool ray_triangle_intersection(IN Ray& input_ray, IN Triangle& triangle, OUT double& t,
	OUT double position[3]) {
	// Moller¨CTrumbore intersection
	double* p1 = triangle.v[0].position;
	double* p2 = triangle.v[1].position;
	double* p3 = triangle.v[2].position;

	double E1[3] = { p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2] };
	double E2[3] = { p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2] };
	double S[3] = {
		input_ray.ray_source[0] - p1[0],
		input_ray.ray_source[1] - p1[1],
		input_ray.ray_source[2] - p1[2]
	};
	double S1[3];
	cross_product(input_ray.ray_direction, E2, S1);
	double S2[3];
	cross_product(S, E1, S2);
	double Div = dot_product(S1, E1);
	if (Div < 0.000001 && Div > -0.000001)
		return false;
	double Mul = 1.0 / Div;
	t = dot_product(S2, E2) * Mul;
	if (t < 0.00001)
		return false;
	double b1 = dot_product(S1, S) * Mul;
	if (b1 < 0)
		return false;
	double b2 = dot_product(S2, input_ray.ray_direction) * Mul;
	if (b2 < 0 || 1 - b1 - b2 < 0)
		return false;
	// calculate the intersection position
	position[0] = input_ray.ray_source[0] + input_ray.ray_direction[0] * t;
	position[1] = input_ray.ray_source[1] + input_ray.ray_direction[1] * t;
	position[2] = input_ray.ray_source[2] + input_ray.ray_direction[2] * t;
	return true;
}

void sphere_normal(IN Sphere sphere, IN double intersection_pos[3], OUT double normal[3]) {
	double* centroid = sphere.position;
	double r_1 = 1 / sphere.radius;
	normal[0] = (intersection_pos[0] - centroid[0]) * r_1;
	normal[1] = (intersection_pos[1] - centroid[1]) * r_1;
	normal[2] = (intersection_pos[2] - centroid[2]) * r_1;
}

double triangle_area(IN double p1[3], IN double p2[3], IN double p3[3]){
	double b_a[3] = { p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2] };
	double c_a[3] = { p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2] };
	double cross[3];
	cross_product(b_a, c_a, cross);
	return 0.5 * vector_length(cross);
}

double distance(IN double P1[3], IN double P2[3]) {
	double vec[3] = {
		P2[0] - P1[0],
		P2[1] - P1[1],
		P2[2] - P1[2]
	};
	return vector_length(vec);
}

// calculate alpha, beta, and gamma
void triangle_interpolation(IN Triangle& triangle, IN double position[3],
	OUT Vertex& vec) {
	Vertex& v0 = triangle.v[0];
	Vertex& v1 = triangle.v[1];
	Vertex& v2 = triangle.v[2];
	double s0_1 = triangle_area(v0.position, v1.position, position);
	double s1_2 = triangle_area(v1.position, v2.position, position);
	double s0_2 = triangle_area(v0.position, v2.position, position);
	double s = triangle_area(v0.position, v1.position, v2.position);
	double sum_area = s0_1 + s1_2 + s0_2;
	// std::cout << s << " " << sum_area << " " << s - sum_area << std::endl;
	sum_area = 1 / sum_area;
	double alpha = sum_area * s1_2;
	double beta = sum_area * s0_2;
	double gamma = sum_area * s0_1;
	vec.shininess = v0.shininess * alpha + v1.shininess * beta + v2.shininess * gamma;
	vec.met = v0.met * alpha + v1.met * beta + v2.met * gamma;
	vec.rou = v0.rou * alpha + v1.rou * beta + v2.rou * gamma;
	memcpy(vec.position, position, sizeof(double) * 3);
	vec.normal[0] = v0.normal[0] * alpha + v1.normal[0] * beta + v2.normal[0] * gamma;
	vec.normal[1] = v0.normal[1] * alpha + v1.normal[1] * beta + v2.normal[1] * gamma;
	vec.normal[2] = v0.normal[2] * alpha + v1.normal[2] * beta + v2.normal[2] * gamma;
	normalize(vec.normal);
	vec.color_diffuse[0] = v0.color_diffuse[0] * alpha + v1.color_diffuse[0] * beta + v2.color_diffuse[0] * gamma;
	vec.color_diffuse[1] = v0.color_diffuse[1] * alpha + v1.color_diffuse[1] * beta + v2.color_diffuse[1] * gamma;
	vec.color_diffuse[2] = v0.color_diffuse[2] * alpha + v1.color_diffuse[2] * beta + v2.color_diffuse[2] * gamma;
	vec.color_specular[0] = v0.color_specular[0] * alpha + v1.color_specular[0] * beta + v2.color_specular[0] * gamma;
	vec.color_specular[1] = v0.color_specular[1] * alpha + v1.color_specular[1] * beta + v2.color_specular[1] * gamma;
	vec.color_specular[2] = v0.color_specular[2] * alpha + v1.color_specular[2] * beta + v2.color_specular[2] * gamma;

}

void random_sample(IN double normal[3], IN int number, OUT double direction[][3]) {
	//for (unsigned int i = 0; i < number; i++) {
	//	double x = (rand() % 1001) / 500.0 - 1.0;
	//	double y = (rand() % 1001) / 500.0 - 1.0;
	//	double z = (rand() % 1001) / 500.0 - 1.0;
	//	float length = sqrt(x * x + y * y + z * z);
	//	if (length > 1) {
	//		i--;
	//		continue;
	//	}
	//	double dir[3] = { x, y, z };
	//	if (dot_product(normal, dir) < 0)
	//		length = -length;
	//	direction[i][0] = x / length;
	//	direction[i][1] = y / length;
	//	direction[i][2] = z / length;
	//}
	std::mt19937_64 rng;
	rng.seed(std::chrono::system_clock::now().time_since_epoch().count());
	for (unsigned int i = 0; i < number; i++) {
		double x = std::uniform_real_distribution<double>(-1, 1)(rng);
		double y = std::uniform_real_distribution<double>(-1, 1)(rng);
		double z = std::uniform_real_distribution<double>(-1, 1)(rng);
		float length = sqrt(x * x + y * y + z * z);
		if (length > 1) {
			i--;
			continue;
		}
		double dir[3] = { x, y, z };
		if (dot_product(normal, dir) < 0)
			length = -length;
		direction[i][0] = x / length;
		direction[i][1] = y / length;
		direction[i][2] = z / length;
	}
}

void random_sample_light(IN int number, OUT double position[][3], 
	OUT double normal[][3], OUT double color[][3], OUT double light_area[]) {
	//srand(time(NULL)); // set random seed
	//for (unsigned int i = 0; i < number; i++) {
	//	double U1 = (rand() % 1001) / 1000.0;
	//	double U2 = (rand() % 1001) / 1000.0;
	//	double U3 = (rand() % 1001) / 1000.0;
	//	int sampledLightID = (int)std::min((int)(num_lights * U1), num_lights - 1);
	//	double* p0 = lights[sampledLightID].p0;
	//	double* p1 = lights[sampledLightID].p1;
	//	double* p2 = lights[sampledLightID].p2;
	//	double* p3 = lights[sampledLightID].p3;
	//	position[i][0] = (1 - U2) * (p0[0] * (1 - U3) + p1[0] * U3) +
	//		U2 * (p2[0] * (1 - U3) + p3[0] * U3);
	//	position[i][1] = (1 - U2) * (p0[1] * (1 - U3) + p1[1] * U3) +
	//		U2 * (p2[1] * (1 - U3) + p3[1] * U3);
	//	position[i][2] = (1 - U2) * (p0[2] * (1 - U3) + p1[2] * U3) +
	//		U2 * (p2[2] * (1 - U3) + p3[2] * U3);
	//	memcpy(normal[i], lights[sampledLightID].normal, sizeof(double) * 3);
	//	memcpy(color[i], lights[sampledLightID].color, sizeof(double) * 3);
	//	light_area[i] = lights[sampledLightID].area;
	//}
	std::mt19937_64 rng;
	rng.seed(std::chrono::system_clock::now().time_since_epoch().count());
	for (unsigned int i = 0; i < number; i++) {
		double U1 = std::uniform_real_distribution<double>(0, 1)(rng);
		double U2 = std::uniform_real_distribution<double>(0, 1)(rng);
		double U3 = std::uniform_real_distribution<double>(0, 1)(rng);
		int sampledLightID = (int)std::min((int)(num_lights * U1), num_lights - 1);
		double* p0 = lights[sampledLightID].p0;
		double* p1 = lights[sampledLightID].p1;
		double* p2 = lights[sampledLightID].p2;
		double* p3 = lights[sampledLightID].p3;
		position[i][0] = (1 - U2) * (p0[0] * (1 - U3) + p1[0] * U3) +
			U2 * (p2[0] * (1 - U3) + p3[0] * U3);
		position[i][1] = (1 - U2) * (p0[1] * (1 - U3) + p1[1] * U3) +
			U2 * (p2[1] * (1 - U3) + p3[1] * U3);
		position[i][2] = (1 - U2) * (p0[2] * (1 - U3) + p1[2] * U3) +
			U2 * (p2[2] * (1 - U3) + p3[2] * U3);
		memcpy(normal[i], lights[sampledLightID].normal, sizeof(double) * 3);
		memcpy(color[i], lights[sampledLightID].color, sizeof(double) * 3);
		light_area[i] = lights[sampledLightID].area;
	}
}

