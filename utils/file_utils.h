#ifndef __PX_MAPPING_LOCALIZATION_FILE_UTILS_H__
#define __PX_MAPPING_LOCALIZATION_FILE_UTILS_H__

#include <algorithm>
#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <sys/stat.h>

bool DoesFileExist(const std::string& name);

bool DoesFileExist(const char* name);

uint64_t GetFileLastModifiedTime(const std::string& file);

uint64_t GetDirLastModifiedTime(const std::string& dir);

int mkdirp(const char* dir, const mode_t mode = S_IRWXU);

bool GetFileContent(std::string file_path, std::string& out_str);
bool GetFileContentByLine(std::string file_path, std::vector<std::string>& out);

bool SaveStringToFile(std::string file_path, std::string& content);

std::vector<std::string> ListAllSubfolders(std::string dir);

std::vector<std::string> ListAllFiles(std::string dir);

std::vector<std::string> ListAllFilesSorted(std::string dir);

uint64_t GetFileLastModifiedTime(const std::string& file);

uint64_t GetDirLastModifiedTime(const std::string& dir);

off_t get_file_size(const char* filename);

#endif