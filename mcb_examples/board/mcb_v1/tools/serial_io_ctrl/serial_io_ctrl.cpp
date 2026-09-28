/*

a little utility to change the RTS and DTR lines of a serial port.

NOTICE
when this program terminates it automatically closes the serial port device.
this will revert the IO line status.  thus, this utility is only useful
together with some other utility which keeps the port opened at all times.

*/

#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include <iostream>
#include <string>

bool set_ios (int fd, bool dtr, bool rts)
{
  int status;

  if (ioctl (fd, TIOCMGET, &status) == -1)
  {
    std::cerr << "get ios NG" << std::endl;
    return false;
  }
/*
  std::cout << "dtr = " << ((status & TIOCM_DTR) != 0)
	    << " rts = " << ((status & TIOCM_RTS) != 0)
	    << std::endl;
*/
  if (dtr)
    status |= TIOCM_DTR;
  else
    status &= ~TIOCM_DTR;

  if (rts)
    status |= TIOCM_RTS;
  else
    status &= ~TIOCM_RTS;

  if (ioctl (fd, TIOCMSET, &status) == -1)
  {
    std::cerr << "set ios NG" << std::endl;
    return false;
  }

  return true;
}

int main (int argc, const char* argv[])
{
  if (argc < 4)
  {
    std::cerr << "expected arguments: <dev path> <dtr status> <rts status>"
	      << std::endl;
    return -1;
  }

  int fd = open (argv[1], O_RDWR | O_NOCTTY);
  if (fd < 0)
  {
    std::cerr << "open NG" << std::endl;
    return -1;
  }

  //std::cout << "fd = " << fd << std::endl;

  set_ios (fd, std::stoi (argv[2]), std::stoi (argv[3]));
  return 0;
}
