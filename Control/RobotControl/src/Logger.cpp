#include "RobotControl/Logger.h"
#include <Windows.h>

logging::Logger logging::logger;



std::vector<std::string> logging::Logger::get_file_names(const std::string& path) {
  std::vector<std::string> names;
  std::string search_path = path + "/*.*";
  WIN32_FIND_DATAA fd;
  HANDLE hFind = ::FindFirstFileA(search_path.c_str(), &fd);
  if (hFind != INVALID_HANDLE_VALUE) {
    do {
      // read all (real) files in current folder
      // , delete '!' read other 2 default folder . and ..
      if (!(fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)) {
        names.push_back(fd.cFileName);
      }
    } while (::FindNextFileA(hFind, &fd));
    ::FindClose(hFind);
  }
  return names;
}
