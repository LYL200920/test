
#include <iostream>
#include <cstdlib>
#include <cstdint>

int main (int argc, const char* argv[])
{
  if (argc < 2)
  {
    std::cerr << "not enough arguments" << std::endl;
    return EXIT_FAILURE;
  }


  size_t byte_count = 0;
  uint8_t tmpbuf[16];
  auto&& fout = std::cout;
  auto&& fin = std::cin;

  std::cout << R"raw(

// generated iCE40 FPGA bitstream data

#include <cstdint>
#include <cstdlib>

extern "C" alignas (32) const uint8_t )raw" << argv[1] << R"raw([] =
{

)raw";


  while (true)
  {
    fin.read ((char*)tmpbuf, sizeof (tmpbuf));

    int sz = fin.gcount ();

    for (int i = 0; i < sz; ++i)
      fout << (unsigned int)tmpbuf[i] << ',';

    fout << '\n';

    byte_count += sz;

    if (sz < sizeof (tmpbuf))
      break;
  }

  fout << R"raw(
};

extern "C" const size_t )raw" << argv[1] << "_size = sizeof (" << argv[1] << ");" << std::endl;

fout << "// byte count = " << byte_count << std::endl;

  return EXIT_SUCCESS;
}
