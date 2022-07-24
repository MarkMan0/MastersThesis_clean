#pragma once

#include <string>
#include <fstream>
#include <chrono>
#include <map>

#include <vector>

#ifndef LOGS_DIR
  #define LOGS_DIR "."
#endif

namespace logging {

  inline const std::string log_dir(LOGS_DIR "/logs/");
  inline std::string log_path(const std::string& name) {
    return log_dir + name + ".txt";
  }

  template <class Iter>
  void log_range(const std::string& name, Iter b, Iter e) {
    std::ofstream out(log_path(name));
    int i = 0;
    while (b != e) {
      out << i++ << ", " << *b << '\n';
      ++b;
    }
  }

  /**
   * Logs one value to a file
   */
  class FileLogger {
  public:
    FileLogger(const std::string& path) : path_(path) {
      out_.open(log_path(path), std::ios::out);
    }

    FileLogger(FileLogger&& old) noexcept : path_(old.path_) {
      old.out_.flush();
      old.out_.close();
      out_.open(log_path(path_), std::ios::app);
    }

    void log(const std::string& what) {
      out_ << time_as_string() << ", " << what << '\n';
    }

    void flush() {
      out_.flush();
    }

    ~FileLogger() {
      if (out_.is_open()) {
        out_.close();
      }
    }

  private:
    std::string time_as_string() const {
      using namespace std::chrono;
      auto tms = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
      return std::to_string(tms);
    }
    const std::string path_;
    std::ofstream out_;
  };

  /**
   * Keeps track of all the log files
   */
  class Logger {
  public:
    Logger() {
      clean_log_dir();
    }

    template <class T>
    void log(const std::string& what, const T& value) {
      log_str(what, std::to_string(value));
    }
    template <>
    void log<std::string>(const std::string& what, const std::string& value) {
      log_str(what, value);
    }


  private:
    static std::vector<std::string> get_file_names(const std::string& path);
    void clean_log_dir() {
      auto files = get_file_names(log_dir);

      for (const auto& name : files) {
        if (name == "README.MD") {
          continue;
        }
        ::remove((log_dir + name).c_str());
      }
    }

    void log_str(const std::string& what, const std::string& value) {
      if (loggers_.find(what) == loggers_.end()) {
        loggers_.emplace(what, FileLogger(what));
      }
      loggers_.at(what).log(value);
      check_flush();
    }
    void check_flush() {
      auto now = std::chrono::steady_clock::now();
      if (now - last_flush_ > std::chrono::seconds(5)) {
        last_flush_ = now;
        for (auto& p : loggers_) {
          p.second.flush();
        }
      }
    }

    std::chrono::steady_clock::time_point last_flush_;
    std::map<std::string, FileLogger> loggers_;
  };

  extern Logger logger;

};  // namespace logging
