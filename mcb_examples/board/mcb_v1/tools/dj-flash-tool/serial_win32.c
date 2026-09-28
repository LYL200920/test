/*
  a minimal win32 port for the serial functions
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <windows.h>

#include "serial.h"


static HANDLE fd;
static int cur_timeout;

static void set_stick_mode (int flash) { }

void serial_set_mode (int sbm) { }

void serial_param (char *parameter) { }

int serial_need_autobaud () { return 1; }

int serial_setup_console (void) { return 0; }

void serial_reset (void) { }

void serial_use_cnvss (int use_it) { }

static void serial_close (void)
{
  CloseHandle (fd);
}

// #define debug_log(...) printf (__VA_ARGS__)
#define debug_log(...) do { } while (0)

void serial_init (const char* port, int baud)
{
  char* port_str = (char*)alloca (strlen (port) + 32);

  strcat (strcpy (port_str, "\\\\.\\"), port);

  fd = CreateFile (port_str, GENERIC_READ | GENERIC_WRITE,
		   0, 0, OPEN_EXISTING, 0, 0);

  if (fd == INVALID_HANDLE_VALUE)
  {
    printf ("error: could not open %s\n", port);
    exit (1);
  }

  atexit (serial_close);

  SetupComm (fd, 1024*1, 1024*1);

  serial_change_baud (baud);

  FlushFileBuffers (fd);

  PurgeComm (fd, PURGE_RXABORT | PURGE_RXCLEAR | PURGE_TXABORT | PURGE_TXCLEAR);

  {
    COMSTAT com_stat;
    DWORD comm_err;
    ClearCommError (fd, &comm_err, &com_stat); 
  }

  serial_set_timeout (1000);
}

void serial_change_baud (int baud)
{
  DCB dcb;
  memset (&dcb, 0, sizeof (DCB));

  dcb.DCBlength = sizeof (DCB);
  dcb.fBinary = 1;
  dcb.BaudRate = baud;
  dcb.ByteSize = 8;
  dcb.StopBits = ONESTOPBIT;

  if (!SetCommState (fd, &dcb))
  {
    printf ("error: SetCommState\n");
    exit (1);
  }
}

void serial_set_timeout (int mseconds)
{
  cur_timeout = mseconds;

  COMMTIMEOUTS com_timeouts;
  if (mseconds <= 0)
  {
    com_timeouts.ReadIntervalTimeout = MAXDWORD;
    com_timeouts.ReadTotalTimeoutConstant = 0;
    com_timeouts.ReadTotalTimeoutMultiplier = 0;
  }
  else
  {
    com_timeouts.ReadTotalTimeoutConstant = mseconds;
    com_timeouts.ReadIntervalTimeout = 0;
    com_timeouts.ReadTotalTimeoutMultiplier = 0;
  }

  com_timeouts.WriteTotalTimeoutMultiplier = 0;
  com_timeouts.WriteTotalTimeoutConstant = 0;

  if (!SetCommTimeouts (fd, &com_timeouts))
  {
    printf ("error: SetCommTimeouts\n");
    exit (1);
  }
}

void serial_pause (int msec)
{
  FlushFileBuffers (fd);
  if (msec > 0)
    Sleep (msec);
}

void serial_sync (void)
{
  FlushFileBuffers (fd);
}

void serial_write (unsigned char ch)
{
  DWORD bytes_written = 0;
  if (!WriteFile (fd, &ch, 1, &bytes_written, NULL))
  {
    printf ("error: WriteFile\n");
    exit (1);
  }
  if (bytes_written != 1)
  {
    printf ("error: bytes_written\n");
    exit (1);
  }
}

void serial_write_block (unsigned char *ch, int len)
{
  DWORD bytes_written = 0;
  if (!WriteFile (fd, ch, len, &bytes_written, NULL))
  {
    printf ("error: WriteFile\n");
    exit (1);
  }
  if (bytes_written != len)
  {
    printf ("error: bytes_written\n");
    exit (1);
  }

  FlushFileBuffers (fd);
}

void
serial_write_string (char *ch)
{
  while (*ch)
    serial_write(*ch++);
  FlushFileBuffers (fd);
}


void serial_drain (void)
{
  int old_timeout = cur_timeout;

  serial_set_timeout (1);
  while (serial_read () != -1) { }

  serial_set_timeout (old_timeout);
}

int serial_read (void)
{
  unsigned char ch;
  DWORD bytes_read = 0;

  ReadFile (fd, &ch, 1, &bytes_read, NULL);

  return bytes_read != 1 ? -1 : ch;
}

int serial_ready (void)
{
  DWORD com_err;
  COMSTAT com_stat;

  ClearCommError (fd, &com_err, &com_stat);

  return com_stat.cbInQue > 0;
}

