#include "stdafx.h"
#include "FileReading.h"

#ifndef _WIN32
#include <dirent.h>
#include <sys/stat.h>
#endif

void getAllFiles(string path, vector<string> &files) {
#ifdef _WIN32
    // Original Windows-specific implementation
    long    hFile = 0;
    struct _finddata_t fileinfo;
    string p;
    if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1) {
        do {
            if ((fileinfo.attrib &  _A_SUBDIR)) {
                if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
                {
                    files.push_back(p.assign(path).append("\\").append(fileinfo.name));
                    getAllFiles(p.assign(path).append("\\").append(fileinfo.name), files);
                }
            }
            else {
                files.push_back(p.assign(path).append("\\").append(fileinfo.name));
            }
        } while (_findnext(hFile, &fileinfo) == 0);
        _findclose(hFile);
    }
#else
    // POSIX implementation
    DIR *dir;
    struct dirent *ent;
    struct stat st;
    if ((dir = opendir(path.c_str())) != NULL) {
        while ((ent = readdir(dir)) != NULL) {
            string file_name = ent->d_name;
            string full_path = path + "/" + file_name;
            if (file_name == "." || file_name == "..") {
                continue;
            }
            if (stat(full_path.c_str(), &st) == 0) {
                if (S_ISDIR(st.st_mode)) {
                    // Recursively get files from subdirectories
                    getAllFiles(full_path, files);
                } else {
                    files.push_back(full_path);
                }
            }
        }
        closedir(dir);
    }
#endif
}

void getAllFormatFiles(string path, vector<string>& files, string format) {
#ifdef _WIN32
    // Original Windows-specific implementation
    long   hFile = 0;
    struct _finddata_t fileinfo;
    string p;
    if ((hFile = _findfirst(p.assign(path).append("\\*" + format).c_str(), &fileinfo)) != -1) {
        do {
            if ((fileinfo.attrib &  _A_SUBDIR)) {
                if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
                {
                    getAllFormatFiles(p.assign(path).append("\\").append(fileinfo.name), files, format);
                }
            }
            else {
                files.push_back(p.assign(path).append("\\").append(fileinfo.name));
            }
        } while (_findnext(hFile, &fileinfo) == 0);
        _findclose(hFile);
    }
#else
    // POSIX implementation
    DIR *dir;
    struct dirent *ent;
    struct stat st;
    if ((dir = opendir(path.c_str())) != NULL) {
        while ((ent = readdir(dir)) != NULL) {
            string file_name = ent->d_name;
            string full_path = path + "/" + file_name;
            if (file_name == "." || file_name == "..") {
                continue;
            }
            if (stat(full_path.c_str(), &st) == 0) {
                if (S_ISDIR(st.st_mode)) {
                    getAllFormatFiles(full_path, files, format);
                } else {
                    if (file_name.length() >= format.length() &&
                        file_name.compare(file_name.length() - format.length(), format.length(), format) == 0) {
                        files.push_back(full_path);
                    }
                }
            }
        }
        closedir(dir);
    }
#endif
}