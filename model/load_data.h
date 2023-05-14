#pragma once
#ifndef _LOAD_DATA_H_
#define _LOAD_DATA_H_
#ifdef WIN32
#define strcasecmp _stricmp
#endif

extern bool verbose;

int loadScenePhong(char* argv);

int loadSceneBRDF(char* argv);

void save_jpg(const char* filename);

void statistic();

void convert_to_obj();
#endif
