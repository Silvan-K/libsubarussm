#include "ECUPort.hh"
#include "Exceptions.hh"

// C library headers
#include <string.h>
#include <iostream>
#include <cstddef>
#include <assert.h>

// Linux headers
#include <fcntl.h>   // File controls (O_RDWR, ...)
#include <errno.h>   // "errno" and strerror()
#include <termios.h> // POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()

namespace SSM {

  ECUPort::ECUPort(const std::string& device_file_path)
  {
    // Open device file
    m_file = open(device_file_path.c_str(), O_RDWR);
    if (m_file<0) throw(DeviceException(strerror(errno)));

    // Configure port
    configurePort(m_file);

    // Initialize communication with ECU
    Bytes response = ECUInit();
  }

  ECUPort::~ECUPort()
  {
    close(m_file);
  }

  Bytes ECUPort::ECUInit() const
  {
    flushBuffer();
    Bytes request = {HEADER, DEST_ECU, SRC_DIAG, Byte(0x01), ECU_INIT};
    request.push_back(checksum(request));
    sendRequest(request);
    return readECUPacket();
  }

  Bytes ECUPort::readBytes(int num_bytes) const
  {
    Bytes response(num_bytes);
    int bufsize  = sizeof(Bytes::value_type)*response.size();
    int num_read = 0;
    
    Bytes::iterator offset = response.begin();
    while(num_read < num_bytes)
      {
	const int n = read(m_file, &(*offset), bufsize);

	if (n<0) throw(DeviceException(strerror(errno)));
	if (n==0) throw(ReadTimeout("Port read timed out"));

	offset   += n;
	num_read += n;
      }
    return response;
  }

  void ECUPort::flushBuffer() const
  {
    Bytes buffer(256);
    const int bufsize = sizeof(Bytes::value_type)*buffer.size();
    while (read(m_file, buffer.data(), bufsize) > 0) {}
  }
  
  void ECUPort::sendRequest(const Bytes& request) const
  {
    write(m_file, request.data(), request.size()*sizeof(Bytes::value_type));
    Bytes echo = readBytes(request.size());
    if(echo != request) 
      throw(UnexpectedResponse("ECU didn't echo request"));
  }

  Bytes ECUPort::readECUPacket() const
  {
    // Read header of packet: -header byte,
    //                        -destination byte
    //                        -source byte
    //                        -datasize byte
    Bytes header = readBytes(4);
    uint8_t dsize = std::to_integer<uint8_t>(header.back());
    if(header.front() != HEADER) 
      throw(UnexpectedResponse("Response header missing"));

    // Now that dsize is known read response body (+1 for checksum byte)
    Bytes body = readBytes(dsize+1);

    // Validate checksum
    uint8_t csum = std::to_integer<uint8_t>(body.back());
    header.insert(header.end(), body.begin(), body.end()-1);
    if(csum != std::to_integer<uint8_t>(checksum(header)))
      throw(UnexpectedResponse("Checksum validation failed"));

    // Not sure what first byte in response is, discard and return
    body.erase(body.begin());
    return body;
  }

  Byte ECUPort::checksum(const Bytes& input) const
  {
    uint8_t sum = 0;
    for(Bytes::const_iterator it = input.begin(); it != input.end(); ++it)
      sum += std::to_integer<uint8_t>(*it);
    return Byte(sum);
  }

  Bytes ECUPort::buildReadRequest(const Observables& observables,
			       bool continuous) const
  {
    // Add all read addresses to a byte vector
    Bytes addresses;
    for(const auto& obs : observables)
      for(const auto& addr: obs->addresses())
	for(const auto& addr_byte: addr)
	  addresses.push_back(addr_byte);
    
    // Build the header of read request. Datasize is number of addresses plus 2
    // since CMD_READ and CONTIN_RESP/SINGLE_RESP bytes are to be included (not
    // the checksum byte though)
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

  Values ECUPort::singleRead(const Observables& observables) const
  {
    sendRequest(buildReadRequest(observables, false));

    Bytes response = readECUPacket();
    Bytes::const_iterator it = response.begin();
    Values values;
    for(const auto& obs: observables)
      {
	values.push_back(obs->convert(it));
	it += obs->numBytes();
      }
    
    return values;
  }

  void ECUPort::continuousRead(const Observables& observables,
			    ReadValueCallback callback) const
  {
    sendRequest(buildReadRequest(observables, true));

    Bytes response(0);
    Values values(observables.size());
    Bytes::const_iterator it;

    while(true)
      {
	response = readECUPacket();

	it = response.begin();
	for(int i(0); i< observables.size(); i++)
	  {
	    values[i] = observables[i]->convert(it);
	    it += observables[i]->numBytes();
	  }
	if(callback(observables, values)) return;
      }
  }

  void ECUPort::configurePort(const int& file)
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
    tty.c_lflag &= ~ICANON;        // Disable canonical mode
    tty.c_lflag &= ~ECHO;          // Disable echo
    tty.c_lflag &= ~ECHOE;         // Disable erasure
    tty.c_lflag &= ~ECHONL;        // Disable new-line echo
    tty.c_lflag &= ~ISIG;          // Disable INTR, QUIT and SUSP
    tty.c_iflag &= ~IXON;          // Turn off software flow control
    tty.c_iflag &= ~IXOFF;
    tty.c_iflag &= ~IXANY;
    tty.c_iflag &= ~IGNBRK;        // Don't handle incoming bytes, get raw data
    tty.c_iflag &= ~BRKINT;
    tty.c_iflag &= ~PARMRK;
    tty.c_iflag &= ~ISTRIP;
    tty.c_iflag &= ~INLCR;
    tty.c_iflag &= ~IGNCR;
    tty.c_iflag &= ~ICRNL;

    tty.c_cc[VTIME] = 20;     // Return as soon as any data is received,
    tty.c_cc[VMIN]  = 0;      // subject to timeout of 2s (20 deciseconds)
    tty.c_oflag &= ~OPOST;    // No special interpretation of output bytes
    tty.c_oflag &= ~ONLCR;    // Noconversion of newline to carriage return/line feed
    cfsetispeed(&tty, B4800); // Set Baud rate to 4800

    // Pass settings back to port
    if (tcsetattr(file, TCSANOW, &tty))
      throw(DeviceException(strerror(errno)));
  }
}
