#include "utils/string_utils.h"

#include <iomanip>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

std::vector<std::string> SplitString(const std::string& stringToSplit, const std::string& regexPattern)
{
    std::vector<std::string> result;

    const std::regex           rgx(regexPattern);
    std::sregex_token_iterator iter(stringToSplit.begin(), stringToSplit.end(), rgx, -1);

    for (std::sregex_token_iterator end; iter != end; ++iter) {
        result.push_back(iter->str());
    }

    return result;
}

// this can lift up to common level method
std::string trim(std::string& s)
{
    if (s.empty()) {
        return s;
    }
    s.erase(0, s.find_first_not_of(" "));
    s.erase(s.find_last_not_of(" ") + 1);

    return s;
}

bool starts_with(std::string s, std::string sub)
{
    return s.find(sub) == 0 ? true : false;
}

bool ends_with(std::string s, std::string sub)
{
    return s.rfind(sub) == (s.length() - sub.length()) ? true : false;
}
std::string replaceStringAll(std::string str, const std::string& replace, const std::string& with)
{
    if (!replace.empty()) {
        std::size_t pos = 0;
        while ((pos = str.find(replace, pos)) != std::string::npos) {
            str.replace(pos, replace.length(), with);
            pos += with.length();
        }
    }
    return str;
}

bool compareCharIgnoreCase(char& c1, char& c2)
{
    if (c1 == c2)
        return true;
    else if (std::toupper(c1) == std::toupper(c2))
        return true;
    return false;
}
/*
 * Case Insensitive String Comparision
 */
bool caseInSensStringCompare(std::string str1, std::string str2)
{
    return ((str1.size() == str2.size()) && std::equal(str1.begin(), str1.end(), str2.begin(), &compareCharIgnoreCase));
}

// trim from start (in place)
inline void ltrim(std::string& s)
{
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
}

// trim from end (in place)
inline void rtrim(std::string& s)
{
    s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
}

// trim from start (copying)
inline std::string ltrim_copy(std::string s)
{
    ltrim(s);
    return s;
}

// trim from end (copying)
inline std::string rtrim_copy(std::string s)
{
    rtrim(s);
    return s;
}

// trim from both ends (copying)
inline std::string trim_copy(std::string s)
{
    trim(s);
    return s;
}