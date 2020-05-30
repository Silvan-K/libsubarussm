// C library headers
#include <stdio.h>
#include <string.h>
#include <iostream>

// Linux headers
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()

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


  class UnexpectedResponse : public Exception {
  public:
    UnexpectedResponse(std::string msg) : Exception(msg) {};
  };

  
  class Port {

  public:

    Port(const std::string& device_file_path);
    ~Port();

    const unsigned char* ECUInit();

  private:

    static void configurePort(const int& device_file);
  
    int m_file;
    unsigned char m_readbuf[256];
  };

  
  Port::Port(const std::string& device_file_path)
  {
    // Open device file
    m_file = open(device_file_path.c_str(), O_RDWR);
    if (m_file<0) throw(DeviceException(strerror(errno)));

    // Configure port
    configurePort(m_file);
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

  const unsigned char* Port::ECUInit()
  {
    unsigned char ecu_init[] = {0x80,  // Header
				0x10,  // Destination: ECU
				0xF0,  // Source: diagnostic tool
				0x01,  // Number of bytes sending (excluding
				       // checksum)
				0xBF,  // Command: ECU Init
				(0x80 + 0x10 + 0xF0 + 0x01 + 0xBF) & 0xff}; // Checksum

    write(m_file, ecu_init, sizeof(ecu_init));

    // TODO: wait for expected number of bytes here, subject to timeout
    sleep(1);

    // # of bytes read. 0 if no bytes received, negative to signal error
    const int n = read(m_file, &m_readbuf, sizeof(m_readbuf));

    return m_readbuf;
  }

}

int main(int argc, char** argv)
{
  SSM::Port ecu("/dev/ttyUSB0");

  const unsigned char* response = ecu.ECUInit();

  for(int i(0); i<101; i++)
    std::cout << (int)response[i] << std::endl;

}
