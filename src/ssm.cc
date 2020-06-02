#include "observables.hh"
#include "exceptions.hh"

// C library headers
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <cstddef>
#include <assert.h>

// Linux headers
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()

namespace SSM {

  class Port {

  public:

    Port(const std::string& device_file_path);
    ~Port();

    std::vector<double> singleRead(const Observables& observables) const;
    void continuousRead(const Observables& observables);

  private:

    static void configurePort(const int& device_file);
    Bytes ECUInit() const;
    Bytes readPort(int num_bytes) const;
    Bytes buildReadRequest(const Observables& observables,
			   bool continuous_response) const;
    Byte checksum(const Bytes& input) const;
  
    int m_file;

    static constexpr Byte HEADER      {0x80};  // Package header
    static constexpr Byte DEST_ECU    {0x10};  // Destination: ECU
    static constexpr Byte SRC_DIAG    {0xF0};  // Source: diagnostic tool
    static constexpr Byte ECU_INIT    {0xBF};  // Command: ECU init
    static constexpr Byte CMD_READ    {0xA8};  // Read parameter(s)
    static constexpr Byte SINGLE_RESP {0x00};  // Single response
    static constexpr Byte CONTIN_RESP {0x01};  // Continuous response
  };

  
  Port::Port(const std::string& device_file_path)
  {
    // Open device file
    m_file = open(device_file_path.c_str(), O_RDWR);
    if (m_file<0) throw(DeviceException(strerror(errno)));

    // Configure port
    configurePort(m_file);

    // Initialize communication with ECU
    Bytes response = ECUInit();
  }

  
  Port::~Port()
  {
    close(m_file);
  }


  void Port::configurePort(const int& file)
  {
    // Read in existing settings
    struct termios tty; memset(&tty, 0, sizeof tty);
    if(tcgetattr(file, &tty)) throw(DeviceException(strerror(errno)));

    // Modify settings
    tty.c_cflag &= ~PARENB;        // Clear parity bit: disable parity
    tty.c_cflag &= ~CSTOPB;        // Use one stop bit
    tty.c_cflag |= CS8;            // 8 bits per byte
    tty.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    tty.c_lflag &= ~ICANON;        // Disable canonical mode (line-by-line mode)
    tty.c_lflag &= ~ECHO;          // Disable echo
    tty.c_lflag &= ~ECHOE;         // Disable erasure
    tty.c_lflag &= ~ECHONL;        // Disable new-line echo
    tty.c_lflag &= ~ISIG;          // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON  |       // Turn off software flow control
		     IXOFF |
		     IXANY);
    tty.c_iflag &= ~(IGNBRK|BRKINT| // Don't handle incoming bytes, get raw data
		     PARMRK|ISTRIP|
		     INLCR|IGNCR|
		     ICRNL);
    tty.c_cc[VTIME] = 10;          // Return as soon as any data is received. Wait for up to 1s (10 deciseconds)
    tty.c_cc[VMIN]  = 0;
    tty.c_oflag &= ~OPOST;         // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR;         // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS;     // Prevent conversion of tabs to spaces (NOT PRESENT IN LINUX)
    // tty.c_oflag &= ~ONOEOT;     // Prevent removal of C-d chars (0x004) in output (NOT PRESENT IN LINUX)
    // tty.c_oflag &= ~OXTABS;     // Prevent conversion of tabs to spaces (NOT PRESENT IN LINUX)
    // tty.c_oflag &= ~ONOEOT;     // Prevent removal of C-d chars (0x004) in output (NOT PRESENT IN LINUX)
    cfsetispeed(&tty, B4800);      // Set Baud rate to 4800

    // Pass settings back to port
    if (tcsetattr(file, TCSANOW, &tty)) throw(DeviceException(strerror(errno)));
  }


  Bytes Port::ECUInit() const
  {
    Bytes request = {HEADER, DEST_ECU, SRC_DIAG, Byte(0x01), ECU_INIT};
    request.push_back(checksum(request));

    // Response to ECU init is 68 bytes long (experimentally determined)
    write(m_file, request.data(), sizeof(Bytes::value_type)* request.size());
    return readPort(68);
  }

  Bytes Port::readPort(int num_bytes) const
  {
    Bytes response(num_bytes);
    int bufsize  = sizeof(Bytes::value_type)*response.size();
    int num_read = 0;
    
    Bytes::iterator offset = response.begin();
    while(num_read < num_bytes)
      {
	const int n = read(m_file, &(*offset), bufsize);
	offset   += n;
	num_read += n;
      }
    return response;
  }

  Byte Port::checksum(const Bytes& input) const
  {
    uint8_t sum = 0;
    for(Bytes::const_iterator it = input.begin(); it != input.end(); ++it)
      sum += std::to_integer<uint8_t>(*it);
    return Byte(sum);
  }

  Bytes Port::buildReadRequest(const Observables& observables,
			       bool continuous) const
  {
    // Add all read addresses to a byte vector
    Bytes addresses;
    for(const auto& obs : observables)
      for(const auto& addr: obs->addresses())
	for(const auto& addr_byte: addr)
	  addresses.push_back(addr_byte);
    
    // Build the start of the read request. Add 2 to the numbre of bytes
    // representing the addresses because the CMD_READ byte and the
    // CONTIN_RESP/SINGLE_RESP bytes are to be included (not the checksum byte
    // though)
    Bytes request = {HEADER,
		     DEST_ECU,
		     SRC_DIAG,
		     Byte(addresses.size()+2),
		     CMD_READ,
		     (continuous ? CONTIN_RESP : SINGLE_RESP)};

    // Attach the addresses to read
    request.insert(request.end(), addresses.begin(), addresses.end());

    // Add the checksum byte and return
    request.push_back(checksum(request));
    return request;
  }

  std::vector<double> Port::singleRead(const Observables& observables) const
  {
    Bytes request = buildReadRequest(observables, false);
    write(m_file, request.data(), request.size()*sizeof(Bytes::value_type));

    // TODO: wait for expected number of bytes here, subject to timeout
    sleep(1);

    // # of bytes read. 0 if no bytes received, negative to signal error
    Bytes response(256);
    const int n = read(m_file, response.data(),
		       sizeof(Bytes::value_type)*response.size());

    Bytes::const_iterator it = response.begin() + 5 + request.size();
    std::vector<double> ret;
    for(const auto& obs: observables)
      {
	ret.push_back(obs->convert(it));
	it += obs->numBytes();
      }
    
    return ret;
  }

}

int main(int argc, char** argv)
{
  SSM::Port ECU("/dev/ttyUSB0");

  SSM::BatteryVoltage           battery_voltage;
  SSM::EngineSpeed              engine_speed;
  SSM::ManifoldRelativePressure manifold_pressure;

  SSM::Observables observables { &battery_voltage,
				 &engine_speed,
				 &manifold_pressure };

  while(true)
    {
      std::vector<double> values = ECU.singleRead(observables);

      assert(observables.size() == values.size());
      for(int i(0); i<values.size(); i++)
	std::cout << values[i] << " " << observables[i]->unit() << std::endl;
    }
}
