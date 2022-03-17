#include "utils/file_utils.h"

#include "utils/shell_utils.h"
#include "utils/string_utils.h"

#include <glog/logging.h>

#include <fstream>
#include <sstream>

#include <limits.h> /* PATH_MAX */
#include <string.h>
#include <sys/stat.h> /* mkdir(2) */

inline bool DoesFileExistImp(const char* name)
{
    struct stat buffer;
    //   std::cout <<" Checking file exist for file : "<<name<<std::endl;
    return (stat(name, &buffer) == 0);
}

uint64_t GetFileLastModifiedTime(const std::string& file)
{
    struct stat file_stats;
    stat(file.c_str(), &file_stats);
    return file_stats.st_ctime;
}

uint64_t GetDirLastModifiedTime(const std::string& dir)
{
    return GetFileLastModifiedTime(dir);
}

bool DoesFileExist(const std::string& name)
{
    return DoesFileExistImp(name.c_str());
}

bool DoesFileExist(const char* name)
{
    return DoesFileExistImp(name);
}

int mkdirp(const char* dir, const mode_t mode)
{
    struct stat sb;
    // if dir already exists and is a directory
    if (stat(dir, &sb) == 0) {
        if (S_ISDIR(sb.st_mode)) {
            return 0;
        }
        else
            return -1;
    }
    else {
        char   tmp[PATH_MAX];
        size_t len = strnlen(dir, PATH_MAX);
        memcpy(tmp, dir, len);
        // remove trailing slash
        if (tmp[len - 1] == '/') {
            tmp[len - 1] = '\0';
        }
        char* p = strrchr(tmp, '/');
        *p      = '\0';
        int ret = mkdirp(tmp, mode);
        if (ret == 0) {
            return mkdir(dir, mode);
        }
    }
    return 0;
}

bool GetFileContent(std::string file_path, std::string& out_str)
{
    std::ifstream tfi(file_path);
    if (tfi.is_open() == false) {
        return false;
    }
    std::stringstream buffer;
    buffer << tfi.rdbuf();
    out_str = buffer.str();
    tfi.close();
    return true;
}
bool GetFileContentByLine(std::string file_path, std::vector<std::string>& out)
{
    out.clear();
    std::ifstream fi(file_path);
    if (fi.is_open() == false) {
        return false;
    }
    for (std::string line; std::getline(fi, line);) {
        out.push_back(line);
    }
    fi.close();
    return true;
}

bool SaveStringToFile(std::string file_path, std::string& content)
{
    std::ofstream fo;
    fo.open(file_path, std::ios::out | std::ios::trunc);
    if (fo.is_open() == false) {
        LOG(ERROR) << "fail to open file to write: " << file_path;
        return false;
    }

    fo << content;
    fo.close();
    return true;
}

std::vector<std::string> ListAllSubfolders(std::string dir)
{
    std::string cmd = "find " + dir + " -maxdepth 1 -type d";
    // LOG(INFO)<< "Listing all avaialble folder in data uploader with command";
    // LOG(INFO)<<cmd;
    auto results = ExecShell(cmd);
    //  LOG(INFO) << results;
    std::vector<std::string> res = SplitString(results, "[\\r\\n]+");
    //  for (auto st: res) LOG(INFO) << st;
    res.erase(res.begin());

    return res;
}

std::vector<std::string> ListAllFiles(std::string dir)
{
    auto                     results = ExecShell("find " + dir + " -maxdepth 1 -type f");
    std::vector<std::string> res     = SplitString(results, "[\\r\\n]+");
    return res;
}

uint64_t GetSizeOfFolder(const std::string& dir)
{
    std::string cmd     = "du -sk " + dir;
    auto        results = ExecShell(cmd);
    //  LOG(INFO) <<"Command is "<<cmd;
    //  LOG(INFO) <<"result is "<<results;
    try {
        std::string::size_type sz;  // alias of size_t
        uint64_t               size = std::stoll(results, &sz);
        return size;
    }
    catch (...) {
        // LOG(ERROR) <<"Trying to get size of a folder but failed, folder is "<<dir;
        // LOG(ERROR) <<"du command is "<<cmd;
        // LOG(ERROR) <<"result of du command is "<<results;
        return 0;
    }
}

off_t get_file_size(const char* filename)
{
    struct stat sstat;
    stat(filename, &sstat);
    off_t size = sstat.st_size;

    return size;
}

std::vector<std::string> ListAllFilesSorted(std::string dir)
{
    auto res = ListAllFiles(dir);
    std::sort(res.begin(), res.end());
    return res;
}