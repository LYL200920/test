/*

C/C++ runtime and standard library support file for newlib and libstdc++

See also
  http://wiki.osdev.org/Porting_Newlib
  http://www.billgatliff.com/newlib.html
  http://www.cs.ccu.edu.tw/~pahsiung/courses/ese/resources/newlib.pdf
  http://www.embecosm.com/appnotes/ean9/ean9-howto-newlib-1.0.html
*/

#include <sys/stat.h>
#include <sys/time.h>
#include <sys/unistd.h>
#include <errno.h>

#include <cstdio>
#include <cassert>
#include <chrono>

#include <utils/langcomp.hpp>

void board_debug_uart_write (const void* data, unsigned int byte_count);

// if we don't do this ...
#undef errno
extern int errno;

// ... each errno access will be expanded as
//   (*__errno())

extern "C" int _write (int file, const void* ptr, size_t len);


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
// environ
// A pointer to a list of environment variables and their values. 
// For a minimal environment, this empty list is adequate:
//char* _env[1] = { 0 };
//char** environ = _env;


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
// exit
// Terminate the current process.
// extern "C" void _exit (int status);
// extern "C" [[gnu::alias ("_exit")]] void exit (int status);

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
// open
// open a file.
extern "C" [[gnu::alias ("_open")]] int open (int file);
extern "C" [[gnu::used]] int _open (const char *name, int flags, ...)
{
  return -1;
}


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
// close
// Close an open file.
extern "C" [[gnu::alias ("_close")]] int close (int file);
extern "C" [[gnu::used]] int _close (int file)
{
  return -1;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
// execve
// Transfer control to a new process.
extern "C" [[gnu::alias ("_execve")]] int
execve (const char* path, char * const argv[], char * const envp[]);

extern "C" [[gnu::used]] int
_execve (const char* path, char * const argv[], char * const envp[])
{
  errno = ENOMEM;
  return -1;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
// fork
// Create a new process.
extern "C" [[gnu::alias ("_fork")]] int fork (void);
extern "C" [[gnu::used]] int _fork (void)
{
  errno = EAGAIN;
  return -1;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
// fstat
// Status of an open file.
// The 'sys/stat.h' header file required is distributed in the 'include'
// subdirectory for this C library.
extern "C" [[gnu::alias ("_fstat")]] int fstat (int file, struct stat *st);
extern "C" [[gnu::used]] int _fstat (int file, struct stat *st)
{
  // treat all files are as character special devices.
  st->st_mode = S_IFCHR;
  return 0;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
// getpid
// Process-ID; this is sometimes used to generate strings unlikely to conflict
// with other processes.
extern "C" [[gnu::alias ("_getpid")]] int getpid (void);
extern "C" [[gnu::used]] int _getpid (void)
{
  return 1;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
// isatty
// Query whether output stream is a terminal.
extern "C" [[gnu::alias ("_isatty")]] int isatty (int file);
extern "C" [[gnu::used]] int _isatty (int file)
{
  switch (file)
  {
    case STDOUT_FILENO:
    case STDERR_FILENO:
    case STDIN_FILENO:
      return true;
    default:
      //errno = ENOTTY;
      errno = EBADF;
      return false;
  }
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// kill
// Send a signal.
extern "C" [[gnu::alias ("_kill")]] int kill (int pid, int sig);
extern "C" [[gnu::used]] int _kill (int pid, int sig)
{
  errno = EINVAL;
  return -1;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// link
// Establish a new name for an existing file.
extern "C" [[gnu::alias ("_link")]] int link (const char* old_name, const char* new_name);
extern "C" [[gnu::used]] int _link (const char* old_name, const char* new_name)
{
  errno = EMLINK;
  return -1;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// lseek
// Set position in a file.
extern "C" [[gnu::alias ("_lseek")]] off_t lseek (int file, off_t ptr, int dir);
extern "C" [[gnu::used]] off_t _lseek (int file, off_t ptr, int dir)
{
  return 0;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// read
// Read a character to a file. `libc' subroutines will use this system routine
// for input from all files, including stdin.  Returns -1 on error or blocks
// until the number of characters have been read.
extern "C" [[gnu::alias ("_read")]] int read (int file, void* ptr, size_t len);
extern "C" [[gnu::used]] int _read (int file, void* ptr, size_t len)
{
  errno = EBADF;
  return -1;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// stat
// Status of a file (by name).
extern "C" [[gnu::alias ("_stat")]] int stat (const char* filepath, struct stat* st);
extern "C" [[gnu::used]] int _stat (const char* filepath, struct stat* st)
{
  st->st_mode = S_IFCHR;
  return 0;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// times
// Timing information for current process.
extern "C" [[gnu::alias ("_times")]] clock_t times (struct tms *buf);
extern "C" clock_t _times (struct tms *buf)
{
  return -1;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// unlink
// Remove a file's directory entry.
extern "C" [[gnu::alias ("_unlink")]] int unlink (const char *name);
extern "C" [[gnu::used]] int _unlink (const char *name)
{
  errno = ENOENT;
  return -1;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// wait
// Wait for a child process.
extern "C" [[gnu::alias ("_wait")]] int wait (int *status);
extern "C" [[gnu::used]] int _wait (int *status)
{
  errno = ECHILD;
  return -1;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// write
// Write a character to a file. `libc' subroutines will use this system
// routine for output to all files, including stdout
// Returns -1 on error or number of bytes sent.
[[gnu::weak]] void logging_hooks_write (const void* data, size_t len)
{
}

extern "C" [[gnu::alias ("_write")]] int write (int file, const void* ptr, size_t len);
extern "C" [[gnu::used]] int _write (int file, const void* ptr, size_t len)
{
  switch (file)
  {
  default:
    errno = EBADF;
    return -1;

  case STDOUT_FILENO:
  case STDERR_FILENO:
    board_debug_uart_write (ptr, len);
    logging_hooks_write (ptr, len);
    return len;
  }
}

// variables defined by the linker script
extern uintptr_t _ram_start;
extern uintptr_t _ram_end;
extern uintptr_t data;
extern uintptr_t edata;
extern uintptr_t bss;
extern uintptr_t ebss;
extern uintptr_t _text_start;
extern uintptr_t _text_end;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// sbrk
// Increase program data space.
// Malloc and related functions depend on this

static uintptr_t g_cur_brk = (uintptr_t)&_ram_start;

extern "C" [[gnu::alias ("_sbrk")]] void* sbrk (ptrdiff_t sz);
extern "C" void* _sbrk (ptrdiff_t sz)
{
  if (g_cur_brk + sz > (uintptr_t)&_ram_end)
  {
    //printf ("sbrk NG %u\n", sz);
    errno = ENOMEM;
    return (void*)-1;
  }

  auto r = g_cur_brk;
  g_cur_brk += sz;
  return (void*)r;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// override gettimeofday
// this is necessary for std::chrono

// FIXME: also implement CLOCKS_PER_SEC and std::clock

extern "C" [[gnu::alias ("_gettimeofday")]] int gettimeofday (struct timeval* tv, void* tz);
extern "C" int _gettimeofday (struct timeval* tv, void* tz)
{
  if (tv != nullptr)
  {
    auto t = std::chrono::system_clock::now ().time_since_epoch ();
    uint64_t micros =
	std::chrono::duration_cast<std::chrono::microseconds> (t).count ();

    tv->tv_sec = micros / 1000000;
    tv->tv_usec = micros % 1000000;
  }
  return 0;
}

extern "C" [[gnu::alias ("_clock_gettime")]] int clock_gettime (clockid_t clk, struct timespec* res);
extern "C" int _clock_gettime (clockid_t clk, struct timespec* res)
{
  assert_unreachable ();
  return 0;
}

/*
void print_mem_info_1 (const char* n, void* s, void* e)
{
  printf ("%s: %08p - %08p = %u bytes = %u KBytes\n",
	  n, s, e,
	  (unsigned int)((char*)e - (char*)s),
	  (unsigned int)((char*)e - (char*)s) >> 10);
}

void print_mem_info (void)
{
  print_mem_info_1 (".text", &_text_start, &_text_end);
  print_mem_info_1 (".data", &data, &edata);
  print_mem_info_1 (".bss", &bss, &ebss);
  print_mem_info_1 ("RAM", &_ram_start, &_ram_end);
}
*/

// this is copy-pasted from newlib.
// on some targets LTO chokes if __cxa_atexit is not here ...
extern "C"
{

enum __atexit_types
{
  __et_atexit,
  __et_onexit,
  __et_cxa
};

int __register_exitproc (int, void (*fn) (void), void*, void*);

}

extern "C" int __cxa_atexit (void (*destructor)(void*),
			     [[gnu::unused]] void* arg,
			     [[gnu::unused]] void* dso)
{
#ifdef DISABLE_GLOBAL_DTORS
  return 0;
#else
  return __register_exitproc (__et_cxa, (void (*)(void)) destructor, arg, dso);
#endif
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// if C++ exceptions are configured-out, override operator new and operator
// delete implementations.  this results in more compact code (which is
// usually the only reason to turn off C++ exceptions).

#if !__cpp_exceptions

#include <cstdlib>
#include <new>
#include <regex>

[[gnu::cold]] void* operator_new_nothrow (size_t count)
{
  void* r = malloc (count);
  if (r == nullptr)
    if (auto f = std::get_new_handler ())
    {
      f ();
      r = malloc (count);
    }

  return r;
}

void* operator new (size_t count, const std::nothrow_t&)
{
  return operator_new_nothrow (count);
}

void* operator new (size_t count)
{
  return operator_new_nothrow (count);
}

void* operator new [] (size_t count)
{
  return operator_new_nothrow (count);
}

void* operator new [] (size_t count, const std::nothrow_t&)
{
  return operator_new_nothrow (count);
}

// it's a bit stupid, but aliases have to be _defined_ in the same translation
// unit for the compiler.  so we can't alias directly to "abort" here.
extern "C" [[noreturn, gnu::cold]] void __do_abort (void) { abort (); }

extern "C" [[noreturn, gnu::alias ("__do_abort")]] void __cxa_throw (void* thrown_exception, struct std::type_info * tinfo, void (*dest)(void*));
extern "C" [[noreturn, gnu::alias ("__do_abort")]] void __cxa_rethrow (void);
extern "C" [[noreturn, gnu::alias ("__do_abort")]] void* __cxa_allocate_exception (std::size_t thrown_size);
extern "C" [[noreturn, gnu::alias ("__do_abort")]] void __cxa_free_exception (void *vptr);

struct __cxa_dependent_exception;
extern "C" [[noreturn, gnu::alias ("__do_abort")]] __cxa_dependent_exception* __cxa_allocate_dependent_exception (void);

extern "C" [[noreturn, gnu::alias ("__do_abort")]]  void __cxa_free_dependent_exception (__cxa_dependent_exception*);

struct _Unwind_Exception;
extern "C" [[noreturn, gnu::alias ("__do_abort")]] void __cxa_call_terminate(_Unwind_Exception* ue_header) throw ();
extern "C" [[noreturn, gnu::weak, gnu::alias ("__do_abort")]] void __cxa_call_unexpected(void* exc_obj_in);
extern "C" [[noreturn, gnu::alias ("__do_abort")]] void* __cxa_get_exception_ptr(void*);
extern "C" [[noreturn, gnu::alias ("__do_abort")]] void* __cxa_begin_catch (void*);
extern "C" [[noreturn, gnu::alias ("__do_abort")]] void __cxa_end_catch (void);

namespace std
{
bool uncaught_exception() throw () { return false; }
int uncaught_exceptions() throw () { return 0; }

// the code gets a bit smaller if we don't alias to abort, but rather
// override with empty functions.  however, it's better to have the thing
// stop in some way should it accidentally run into one of these functions.
[[noreturn, gnu::alias ("__do_abort")]] void __throw_bad_exception (void);
[[noreturn, gnu::alias ("__do_abort")]] void __throw_bad_alloc (void);
[[noreturn, gnu::alias ("__do_abort")]] void __throw_bad_cast (void);
[[noreturn, gnu::alias ("__do_abort")]] void __throw_bad_typeid (void);
[[noreturn, gnu::alias ("__do_abort")]] void __throw_logic_error (const char*);
[[noreturn, gnu::alias ("__do_abort")]] void __throw_domain_error (const char*);
[[noreturn, gnu::alias ("__do_abort")]] void __throw_invalid_argument (const char*);
[[noreturn, gnu::alias ("__do_abort")]] void __throw_length_error (const char*);
[[noreturn, gnu::alias ("__do_abort")]] void __throw_out_of_range (const char*);
[[noreturn, gnu::alias ("__do_abort")]] void __throw_out_of_range_fmt(const char*, ...);
[[noreturn, gnu::alias ("__do_abort")]] void __throw_runtime_error(const char*);
[[noreturn, gnu::alias ("__do_abort")]] void __throw_range_error (const char*);
[[noreturn, gnu::alias ("__do_abort")]] void __throw_overflow_error (const char*);
[[noreturn, gnu::alias ("__do_abort")]] void __throw_underflow_error (const char*);
[[noreturn, gnu::alias ("__do_abort")]] void __throw_ios_failure (const char*);
[[noreturn, gnu::alias ("__do_abort")]] void __throw_system_error(int);
[[noreturn, gnu::alias ("__do_abort")]] void __throw_future_error(int);
[[noreturn, gnu::alias ("__do_abort")]] void __throw_bad_function_call (void);
[[noreturn, gnu::alias ("__do_abort")]] void __throw_regex_error(regex_constants::error_type);

} // namespace std

#endif // __cpp_exceptions

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

