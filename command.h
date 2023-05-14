#pragma once
#ifndef _COMMAND_H_
#define _COMMAND_H_
#include <vector>
#include <string>
#include <regex>
#include <iostream>

enum class RenderMode {
	PhoneShading,
	BRDF
};

struct StartParameter {
	char file_path[256];
	char img_savepath[256];
	RenderMode mode;
};

extern StartParameter startParameter;
StartParameter startParameter;

extern void cmd_parameter(int argc, char** argv) {
	if (argc == 3) {
		memcpy(startParameter.file_path, argv[1], sizeof(char) * strlen(argv[1]));
		memcpy(startParameter.img_savepath, argv[2], sizeof(char) * strlen(argv[2]));
		startParameter.mode = RenderMode::PhoneShading;
	}
	else if (argc == 4) {
		memcpy(startParameter.file_path, argv[1], sizeof(char) * strlen(argv[1]));
		memcpy(startParameter.img_savepath, argv[2], sizeof(char) * strlen(argv[2]));
		if (strcmp(argv[3], "phong") == 0) {
			startParameter.mode = RenderMode::PhoneShading;
		}
		else if (strcmp(argv[3], "brdf") == 0) {
			startParameter.mode = RenderMode::BRDF;
		}
		else {
			printf("parameter 3: optional paremeter. There are 2 options:\n"
				"	\"phong\": the scene will render by Phong shading\n"
				"	\"brdf\": the scene will render by BRDF\n"
				"	Phong is the default option. Mind that the data file you provided "
				"must be the same type as the render option.\n"
				"	The parameter you provide is not correct, the default phong mode will be used");
			startParameter.mode = RenderMode::PhoneShading;
		}
	}
	else {
		printf("Please provide 2 or 3 parameters:\n"
			"parameter 1: data file path, with file extension .scene\n"
			"parameter 2: image save path, with file extension .jpg\n"
			"parameter 3: optional paremeter. There are 2 options:\n"
			"	\"phong\": the scene will render by Phong shading\n"
			"	\"brdf\": the scene will render by BRDF\n"
			"	Phong is the default option. Mind that the data file you provided "
			"must be the same type as the render option.");
		exit(0);
	}
}

#endif