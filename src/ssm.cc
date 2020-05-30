// C library headers
#include <stdio.h>
#include <string.h>
#include <iostream>

// Linux headers
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()

int main(int argc, char** argv)
{
  const int serial_port = open("/dev/ttyUSB0", O_RDWR);
  
  if (serial_port < 0)
    std::cout << "Error " << errno
	      << " from open: " << strerror(errno)
	      << std::endl;

  // Create new termios struct, fill with zeroes
  struct termios tty; memset(&tty, 0, sizeof tty);

  // Read in existing settings, and handle any error
  if(tcgetattr(serial_port, &tty) != 0) {
    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
  }

  tty.c_cflag &= ~PARENB;        // Clear parity bit, disabling parity
  tty.c_cflag &= ~CSTOPB;        // Clear stop field, use only one stop bit
  tty.c_cflag |= CS8;            // 8 bits per byte
  tty.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
  tty.c_lflag &= ~ICANON;        // Disable canonical mode (line-by-line mode)
  tty.c_lflag &= ~ECHO;          // Disable echo
  tty.c_lflag &= ~ECHOE;         // Disable erasure
  tty.c_lflag &= ~ECHONL;        // Disable new-line echo
  tty.c_lflag &= ~ISIG;          // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off software flow control
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // No
								   // handling
								   // of
								   // received
								   // bytes,
								   // want
								   // raw
								   // data
  tty.c_oflag &= ~OPOST;     // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR;     // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT IN LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT IN LINUX)
  tty.c_oflag &= ~OPOST;     // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR;     // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT IN LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT IN LINUX)
  
  tty.c_cc[VTIME] = 10;      // Return as soon as any data is received. Wait for up to 1s (10 deciseconds)
  tty.c_cc[VMIN]  = 0;

  cfsetispeed(&tty, B4800);  // Set Baud rate to 4800

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
  }

  unsigned char msg[] = { 'H', 'e', 'l', 'l', 'o', '\r' };
  write(serial_port, "Hello, world!", sizeof(msg));

  // Allocate memory for read buffer, set size according to your needs
  char read_buf [256];
  memset(&read_buf, '\0', sizeof(read_buf));

  // # of bytes read. 0 if no bytes received, negative to signal error
  int n = read(serial_port, &read_buf, sizeof(read_buf));

  close(serial_port);
}
