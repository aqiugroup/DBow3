#ifndef __PX_MAPPING_LOCALIZATION_STRING_UTILS_H__
#define __PX_MAPPING_LOCALIZATION_STRING_UTILS_H__

#include <chrono>
#include <string>
#include <vector>

std::vector<std::string> SplitString(const std::string& stringToSplit, const std::string& regexPattern);
std::string              trim(std::string& s);

bool starts_with(std::string s, std::string sub);
bool ends_with(std::string s, std::string sub);

std::string replaceStringAll(std::string str, const std::string& replace, const std::string& with);

/*
 * Case Insensitive String Comparision
 * true  : same
 * false : not same
 */
bool caseInSensStringCompare(std::string str1, std::string str2);

// trim from start (in place)
void ltrim(std::string& s);

// trim from end (in place)
void rtrim(std::string& s);

// trim from start (copying)
std::string ltrim_copy(std::string s);

// trim from end (copying)
std::string rtrim_copy(std::string s);

// trim from both ends (copying)
std::string trim_copy(std::string s);

#endif