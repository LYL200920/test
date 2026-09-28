/*
original example taken from:
  http://www.gnu.org/software/libc/manual/html_node/Server-Example.html

modified for C++
*/

#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <cassert>
#include <iostream>
#include <string>

constexpr unsigned int PORT = 5555;
constexpr unsigned int MAXMSG = 512;

int read_from_client (int filedes)
{
  char buffer[MAXMSG];
  int nbytes = read (filedes, buffer, MAXMSG);

  if (nbytes < 0)
    assert (false && "read NG");

  else if (nbytes == 0)
  {
    // end-of-file (EOF)
    return -1;
  }
  else
  {
    // got some data data read
    std::cout << "received " << nbytes << " bytes: "
	      << std::string (buffer, nbytes) << std::endl;
    return 0;
  }
}

int make_socket (uint16_t port)
{
  // Create the socket.
  int sock = socket (PF_INET, SOCK_STREAM, 0);
  if (sock < 0)
    assert (false && "socket NG");

  // Bind socket to an address and port.
  struct sockaddr_in addr;
  addr.sin_family = AF_INET;
  addr.sin_port = htons (port);
  addr.sin_addr.s_addr = htonl (INADDR_ANY);

  if (bind (sock, (struct sockaddr*)&addr, sizeof (addr)) < 0)
    assert (false && "bind NG");

  return sock;
}

int
main (void)
{
  // Create the socket and set it up to accept connections.
  int sock = make_socket (PORT);
  if (listen (sock, 1) < 0)
    assert (false && "listen NG");

  // Initialize the set of active sockets.
  fd_set active_fd_set;
  FD_ZERO (&active_fd_set);
  FD_SET (sock, &active_fd_set);

  while (true)
  {
    // Block until input arrives on one or more active sockets.
    fd_set read_fd_set = active_fd_set;
    if (select (FD_SETSIZE, &read_fd_set, nullptr, nullptr, nullptr) < 0)
      assert (false && "select NG");

    // Service all the sockets with input pending.
    for (int i = 0; i < FD_SETSIZE; ++i)
      if (FD_ISSET (i, &read_fd_set))
      {
        if (i == sock)
	{
	  // Connection request on original socket.
	  struct sockaddr_in client_addr;
	  socklen_t client_addr_size = sizeof (client_addr);
	  int new_socket = accept (sock, (struct sockaddr*)&client_addr, &client_addr_size);
	  if (new_socket < 0)
	    assert (false && "accept NG");

	  std::cout << "new connection from "
		    << inet_ntoa (client_addr.sin_addr)
		    << ":" << ntohs (client_addr.sin_port) << std::endl;

	  FD_SET (new_socket, &active_fd_set);
	}
	else
	{
	  // Data arriving on an already-connected socket.
	  if (read_from_client (i) < 0)
	  {
	    close (i);
	    FD_CLR (i, &active_fd_set);
	  }
	}
      }
  }
}
