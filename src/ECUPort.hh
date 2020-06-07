#pragma once

#include "Bytes.hh"
#include "Observables.hh"

#include <vector>
#include <functional>

namespace SSM {

  typedef std::vector<double> Values;

  class ECUPort {

    typedef std::function<int(const Observables& observables,
			      const Values& values)> ReadValueCallback;

  public:

    ECUPort(const std::string& device_file_path);
    ~ECUPort();

    Values singleRead(const Observables& observables) const;
    void continuousRead(const Observables& observables,
			ReadValueCallback callback) const;

  private:

    static void configurePort(const int& device_file);
    void  sendRequest(const Bytes& request, int max_try=3) const;
    Bytes ECUInit() const;
    Bytes readBytes(int num_bytes) const;
    Bytes readECUPacket(bool echo) const;
    Bytes buildReadRequest(const Observables& observables,
			   bool continuous_response) const;
    Byte  checksum(const Bytes& input) const;
  
    int m_file;

    static constexpr Byte HEADER      {0x80};  // Package header
    static constexpr Byte ECU         {0x10};  // Device: ECU
    static constexpr Byte TOOL        {0xF0};  // Device: diagnostic tool
    static constexpr Byte ECU_INIT    {0xBF};  // Command: ECU init
    static constexpr Byte CMD_READ    {0xA8};  // Read parameter(s)
    static constexpr Byte SINGLE_RESP {0x00};  // Single response
    static constexpr Byte CONTIN_RESP {0x01};  // Continuous response
  };
  
}
