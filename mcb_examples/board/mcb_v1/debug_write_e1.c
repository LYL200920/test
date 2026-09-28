
#if 0

// write a byte of data over the E1 debug adapter to the HEW debug/printf console
static void debug_putchar_e1 (char val)
{
  volatile unsigned long* DBGSTAT = (volatile unsigned long *)0x000840C0;
  volatile unsigned long* FC2E0 = (volatile unsigned long *)0x00084080;

  while (*DBGSTAT & 0x00000100) { }

  *FC2E0 = val;
}

// override default putchar standard function.
int putchar (int c)
{
  debug_putchar_e1 (c);
  return c;
}

// override default write standard function.
// (not used with KPIT C lib)
int write (int fd, const void* buf, unsigned int count)
{
  unsigned int countt = count;
  const char* buff = (const char*)buf;

  while (countt > 0)
  {
    debug_putchar_e1 (*buff++);
    --countt;
  }
  return count;
}

#endif