


/*
#include <utils/buffer.hpp>
int main (void)
{
  utils::buffer buf ("asdasd");
  return 0;
}
*/



#include <iostream>
#include <utils/buffer.hpp>
#include <utils/text.hpp>
#include <utils/rodata.hpp>

#include <list>
#include <vector>
#include <string_view>

void print_buffer (const utils::buffer& buf)
{
  const uint8_t* d = (const uint8_t*)buf.data ();
  auto sz = buf.size ();

  std::cout <<	"buffer size: " << sz << ", data (" << (const void*)d << "): ";

  for (decltype (sz) i = 0; i < sz; ++i)
    std::cout << utils::printf_format ("%02x ", d[i]);

  std::cout << std::endl;
}


void test (void)
{
  struct my_data : public utils::buffer_data
  {
    virtual const void* data (void) const override { return nullptr; }
    virtual size_type size (void) const override { return 0; }
  };

  utils::buffer buf (utils::make_ref_counted<my_data> ());
}

int main (void)
{
  std::cout << "vector: " << utils::is_contiguous_container <std::vector<int>>::value << std::endl;
  std::cout << "array: " << utils::is_contiguous_container <std::array<int,10>>::value << std::endl;
  std::cout << "string: " << utils::is_contiguous_container <std::string>::value << std::endl;
  std::cout << "list: " << utils::is_contiguous_container <std::list<int>>::value << std::endl;
  std::cout << "string_view: " << utils::is_contiguous_container <std::string_view>::value << std::endl;
  //std::cout << "x: " << __builtin_constant_p ("bbbbb") << std::endl;

  utils::buffer buf_00 (std::string ("a"));
  utils::buffer buf_01 ("bbbbb");
  utils::buffer buf_02 (std::string_view ("bbbbb"));
  utils::buffer buf_03 (utils::make_array (1));
  utils::buffer buf_04 (utils::buffer::weak, std::string_view ("bbbbb"));
  utils::buffer buf_05 (utils::buffer::weak, "bbbbb");

  print_buffer (buf_00);
  print_buffer (buf_01);
  print_buffer (buf_02);
  print_buffer (buf_03);
  print_buffer (buf_04);
  print_buffer (buf_05);


  std::cout << std::endl;
  return 0;
}



/*
int main (void)
{
  utils::ref_counting_ptr<utils::buffer_data> bleh;
  int bleh2;
  const utils::ref_counting_ptr<utils::buffer_data>& bleh3 = bleh;

  std::cout << "x1: " << utils::is_ref_counting_ptr<decltype (bleh)>::value << std::endl;
  std::cout << "x2: " << utils::is_ref_counting_ptr<decltype (bleh2)>::value << std::endl;
  std::cout << "x3: " << utils::is_ref_counting_ptr<decltype (bleh3)>::value << std::endl;

  return 0;
};
*/

