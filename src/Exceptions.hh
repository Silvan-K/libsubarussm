#pragma once

namespace SSM {

  class Exception : public std::exception {

  public:
    Exception(std::string msg) : m_what(msg) {};
    const char* what() const noexcept { return m_what.c_str(); };

  private:
    std::string m_what;
  };

  class DeviceException : public Exception {
  public:
    DeviceException(std::string msg) : Exception(msg) {};
  };

  class InvalidChecksum : public Exception {
  public:
    InvalidChecksum(std::string msg) : Exception(msg) {};
  };

  class ReadTimeout : public Exception {
  public:
    ReadTimeout(std::string msg) : Exception(msg) {};
  };
}
