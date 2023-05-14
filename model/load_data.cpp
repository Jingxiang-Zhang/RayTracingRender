#include "load_data.h"
#include "scene.h"
#include "../openGL/plot.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "geometry.h"
#include "../jpg/loadJPG.h"

bool verbose = true;

void parse_check(const char* expected, char* found)
{
    if (strcasecmp(expected, found))
    {
        printf("Expected '%s ' found '%s '\n", expected, found);
        printf("Parse error, abnormal abortion\n");
        exit(0);
    }
}

void parse_doubles(FILE* file, const char* check, double p[3])
{
    char str[100];
    fscanf(file, "%s", str);
    parse_check(check, str);
    fscanf(file, "%lf %lf %lf", &p[0], &p[1], &p[2]);
    if (verbose)printf("%s %lf %lf %lf\n", check, p[0], p[1], p[2]);
}

void parse_rad(FILE* file, double* r)
{
    char str[100];
    fscanf(file, "%s", str);
    parse_check("rad:", str);
    fscanf(file, "%lf", r);
    if (verbose)printf("rad: %f\n", *r);
}

void parse_shi(FILE* file, double* shi)
{
    char s[100];
    fscanf(file, "%s", s);
    parse_check("shi:", s);
    fscanf(file, "%lf", shi);
    if (verbose)printf("shi: %f\n", *shi);
}

void parse_rou(FILE* file, double* shi)
{
    char s[100];
    fscanf(file, "%s", s);
    parse_check("rou:", s);
    fscanf(file, "%lf", shi);
    if (verbose)printf("rou: %f\n", *shi);
}

void parse_met(FILE* file, double* shi)
{
    char s[100];
    fscanf(file, "%s", s);
    parse_check("met:", s);
    fscanf(file, "%lf", shi);
    if (verbose)printf("met: %f\n", *shi);
}

int loadSceneBRDF(char* argv) {
    FILE* file = fopen(argv, "r");
    int number_of_objects;
    char type[50];
    Triangle t;
    Sphere s;
    Light l;
    fscanf(file, "%i", &number_of_objects);
    if (verbose) printf("number of objects: %i\n", number_of_objects);
    parse_doubles(file, "amb:", ambient_light);
    parse_doubles(file, "f0:", f0);

    for (int i = 0; i < number_of_objects; i++)
    {
        fscanf(file, "%s\n", type);
        if (verbose) printf("%s\n", type);
        if (strcasecmp(type, "triangle") == 0)
        {
            if (verbose) printf("found triangle\n");
            for (int j = 0; j < 3; j++)
            {
                parse_doubles(file, "pos:", t.v[j].position);
                parse_doubles(file, "nor:", t.v[j].normal);
                parse_doubles(file, "dif:", t.v[j].color_diffuse);
                parse_rou(file, &t.v[j].rou);
                parse_met(file, &t.v[j].met);
            }
            if (num_triangles == MAX_TRIANGLES)
            {
                if (verbose) printf("too many triangles, you should increase MAX_TRIANGLES!\n");
                exit(0);
            }
            triangles[num_triangles++] = t;
        }
        else if (strcasecmp(type, "sphere") == 0)
        {
            if (verbose) printf("found sphere\n");

            parse_doubles(file, "pos:", s.position);
            parse_rad(file, &s.radius);
            parse_doubles(file, "dif:", s.color_diffuse);
            parse_rou(file, &s.rou);
            parse_met(file, &s.met);

            if (num_spheres == MAX_SPHERES)
            {
                if (verbose) printf("too many spheres, you should increase MAX_SPHERES!\n");
                exit(0);
            }
            spheres[num_spheres++] = s;
        }
        else if (strcasecmp(type, "light") == 0)
        {
            if (verbose) printf("found light\n");
            parse_doubles(file, "p0:", l.p0);
            parse_doubles(file, "p1:", l.p1);
            parse_doubles(file, "p2:", l.p2);
            parse_doubles(file, "p3:", l.p3);
            parse_doubles(file, "pos:", l.position);
            parse_doubles(file, "nrm:", l.normal);
            parse_doubles(file, "col:", l.color);
            l.area = std::abs(distance(l.p0, l.p1) * distance(l.p0, l.p2));
            if (num_lights == MAX_LIGHTS)
            {
                if (verbose) printf("too many lights, you should increase MAX_LIGHTS!\n");
                exit(0);
            }
            lights[num_lights++] = l;
        }
        else
        {
            if (verbose) printf("unknown type in scene description:\n%s\n", type);
            exit(0);
        }
    }
    return 0;
}

int loadScenePhong(char* argv)
{
    FILE* file = fopen(argv, "r");
    int number_of_objects;
    char type[50];
    Triangle t;
    Sphere s;
    Light l;
    fscanf(file, "%i", &number_of_objects);

    if(verbose) printf("number of objects: %i\n", number_of_objects);

    parse_doubles(file, "amb:", ambient_light);

    for (int i = 0; i < number_of_objects; i++)
    {
        fscanf(file, "%s\n", type);
        if (verbose) printf("%s\n", type);
        if (strcasecmp(type, "triangle") == 0)
        {
            if (verbose) printf("found triangle\n");
            for (int j = 0; j < 3; j++)
            {
                parse_doubles(file, "pos:", t.v[j].position);
                parse_doubles(file, "nor:", t.v[j].normal);
                parse_doubles(file, "dif:", t.v[j].color_diffuse);
                parse_doubles(file, "spe:", t.v[j].color_specular);
                parse_shi(file, &t.v[j].shininess);
            }

            if (num_triangles == MAX_TRIANGLES)
            {
                if (verbose) printf("too many triangles, you should increase MAX_TRIANGLES!\n");
                exit(0);
            }
            triangles[num_triangles++] = t;
        }
        else if (strcasecmp(type, "sphere") == 0)
        {
            if (verbose) printf("found sphere\n");

            parse_doubles(file, "pos:", s.position);
            parse_rad(file, &s.radius);
            parse_doubles(file, "dif:", s.color_diffuse);
            parse_doubles(file, "spe:", s.color_specular);
            parse_shi(file, &s.shininess);

            if (num_spheres == MAX_SPHERES)
            {
                if (verbose) printf("too many spheres, you should increase MAX_SPHERES!\n");
                exit(0);
            }
            spheres[num_spheres++] = s;
        }
        else if (strcasecmp(type, "light") == 0)
        {
            if (verbose) printf("found light\n");
            parse_doubles(file, "pos:", l.position);
            parse_doubles(file, "col:", l.color);

            if (num_lights == MAX_LIGHTS)
            {
                if (verbose) printf("too many lights, you should increase MAX_LIGHTS!\n");
                exit(0);
            }
            lights[num_lights++] = l;
        }
        else
        {
            if (verbose) printf("unknown type in scene description:\n%s\n", type);
            exit(0);
        }
    }
    return 0;
}

void save_jpg(const char* filename)
{
    printf("Saving JPEG file: %s\n", filename);
    LoadJPG img(WIDTH, HEIGHT, 3, &buffer[0][0][0]);
    if (img.save(filename) != LoadJPG::OK)
        printf("Error in Saving\n");
    else
        printf("File saved Successfully\n");
}

void statistic() {
    double min_x = 100000.0;
    double max_x = -100000.0;
    double min_y = 100000.0;
    double max_y = -100000.0;
    double min_z = 100000.0;
    double max_z = -100000.0;
    for (int i = 0; i < num_spheres; i++) {
        Sphere& sphere = spheres[i];
        min_x = min(min_x, sphere.position[0] - sphere.radius);
        max_x = max(max_x, sphere.position[0] + sphere.radius);
        min_y = min(min_y, sphere.position[1] - sphere.radius);
        max_y = max(max_y, sphere.position[1] + sphere.radius);
        min_z = min(min_z, sphere.position[2] - sphere.radius);
        max_z = max(max_z, sphere.position[2] + sphere.radius);
    }
    for (int i = 0; i < num_triangles; i++) {
        for (int j = 0; j < 3; j++) {
            Vertex& vertex = triangles[i].v[j];
            min_x = min(min_x, vertex.position[0]);
            max_x = max(max_x, vertex.position[0]);
            min_y = min(min_y, vertex.position[1]);
            max_y = max(max_y, vertex.position[1]);
            min_z = min(min_z, vertex.position[2]);
            max_z = max(max_z, vertex.position[2]);
        }
    }
    sceneStatistic.min_x = min_x;
    sceneStatistic.max_x = max_x;
    sceneStatistic.min_y = min_y;
    sceneStatistic.max_y = max_y;
    sceneStatistic.min_z = min_z;
    sceneStatistic.max_z = max_z;
}

void convert_to_obj() {
    for (int i = 0; i < num_spheres; i++) {
        Object obj;
        obj.type = Object::objType::sphere;
        obj.pointer.sphere = &spheres[i];
        obj_list.push_back(obj);
    }
    for (int i = 0; i < num_triangles; i++) {
        Object obj;
        obj.type = Object::objType::triangle;
        obj.pointer.triangle = &triangles[i];
        obj_list.push_back(obj);
    }
}