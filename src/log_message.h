#ifndef BASE_LOGGING_LOG_MESSAGE_H_
#define BASE_LOGGING_LOG_MESSAGE_H_

// #define USE_SYSLOG
#undef USE_SYSLOG


// #define DEBUG

#include <assert.h>
#if !defined(USE_GLOG)

#if defined(LOG)
#undef LOG
#endif

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <sstream>
#include <ostream>
#include <string.h>
#include <pthread.h>
#include <chrono>
#include <iostream>
#include <iomanip>

#if defined(USE_SYSLOG)
#include <syslog.h>
#endif

typedef int LogSeverity;
#if defined(USE_SYSLOG)
const LogSeverity VERBOSE = -1;
const LogSeverity INFO = LOG_INFO;
const LogSeverity WARNING = LOG_WARNING;
const LogSeverity ERROR = LOG_ERR;
const LogSeverity FATAL = LOG_ERR;
#else
const LogSeverity VERBOSE = -1;
const LogSeverity INFO = 1;
const LogSeverity WARNING = 2;
const LogSeverity ERROR = 3;
const LogSeverity FATAL = 4;
#endif

namespace base {

namespace logging {

class LogMessage {
public:
  LogMessage(LogSeverity severity, const std::string &file, const std::string &func, int line)
  : severity_(severity), file_(basename(file.c_str())), func_(func), line_(line) {
    static auto start = std::chrono::high_resolution_clock::now();
    if(severity_ == INFO) {
      stream() << "[" << severity_str[severity_] << "]"
      << "[" << std::hex << std::setw(14) << std::setfill('0') << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count() << "]"
      << "[" << std::hex <<  (uint64_t) pthread_self() << "]" << std::dec;
    }
    else {
      stream() << "[" << severity_str[severity_] << "]"
      << "[" << std::hex << std::setw(14) << std::setfill('0') << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count() << "]"
      << "[" << std::hex <<  (uint64_t) pthread_self() << "]" << std::dec << "[" << file_ << ":" <<line_ << "] ";
    }
  }

  ~LogMessage() {
#if defined(USE_SYSLOG)
    syslog(severity_, "%s",stream_.str().c_str());
#else
    std::cerr << stream_.str() << std::endl;
#endif
  }

  std::ostream &stream() {return stream_;}

private:
  std::ostringstream stream_;
  LogSeverity severity_;
  const std::string file_;
  const std::string func_;
  const unsigned int line_;
  const std::vector<std::string> severity_str{ "", "INFO", "WARNING", "ERROR", "FATAL" };
};

} // namespace logging

} // namespace base

#define LOG(verbose_level) ::base::logging::LogMessage(verbose_level, __FILE__, __func__, __LINE__).stream()


#else
#include <glog/logging.h>
#endif

#include <iostream>
#include <iomanip>

namespace base {

namespace logging {
class Hexdump {
public:
  Hexdump(const std::string &file, const std::string &func, int line, char *data, int len) {
    const int unit = 0x10;
    int lines = len / unit;
    if(len % unit)
      lines += 1;

    for(int i = 0; i < lines; i++) {
      stream_.width(4);
	    stream_.fill('0');
      stream_ << std::hex << (i * unit) << " : ";

      int off = unit * i;
      for(int j = 0; j < unit; j++) {
        char tmp = '.';
        if(((data[off+j] >= 'A') && (data[off+j] <= 'z')) || ((data[off+j] >= '0') && (data[off+j] <= '9'))) {
          tmp = data[off+j];
        }
        stream_ << tmp;
      }

      stream_.width(2);
	    stream_.fill('0');
      for(int j = 0; j < unit; j++) {
        stream_ << std::hex << std::setw(2) << std::uppercase << (int)((unsigned char)data[off+j]);
        stream_ << " ";
      }

      stream_ << "\n";
    }
  }

  ~Hexdump() {
    std::cerr << stream_.str() << std::endl;
  }

  std::ostream &stream() {return stream_;}

private:
  std::ostringstream stream_;
};

} // logging

} // base

#define H(data, len) ::base::logging::Hexdump(__FILE__, __func__, __LINE__, data, len)

#endif // BASE_LOGGING_LOG_MESSAGE_H_
