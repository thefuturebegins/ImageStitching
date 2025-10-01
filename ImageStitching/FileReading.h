#ifndef _FILEREADING_H_
#define _FILEREADING_H_

#ifdef _WIN32
#include <io.h>
#else
#include <dirent.h>
#include <sys/stat.h>
#endif
#include <string>
#include <vector>
#include <iostream>

using namespace std;

void getAllFiles(string path, vector<string> &files);
void getAllFormatFiles(string path, vector<string>& files, string format);

#endif