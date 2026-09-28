
/*

a dummy source file to work around a cmake deficit.

cmake doesn't know how to deal with linker scripts.  when the linker script
is edited, we would like to trigger a re-link of the application.

to accomplish this, this dummy source file gets a dependency set on the
linker script file, and the mcb_lib depends on this file.  this will trigger
a relink.

see also
https://cmake.org/pipermail/cmake/2010-May/037206.html

*/
