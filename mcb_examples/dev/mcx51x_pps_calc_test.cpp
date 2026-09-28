
// can also compile as standalone file to check assembly output
//
// /opt/jz/gcc8-190509/rx-elf/bin/rx-elf-g++ -I../ -m64bit-doubles -mcpu=RX600 -O2 -std=c++14 -S -dp -o - mcx51x_pps_calc_test.cpp
// g++ -I../ -fomit-frame-pointer -std=c++14 -O2 -S -dp -o - mcx51x_pps_calc_test.cpp

namespace std
{
namespace this_thread
{
template <typename T> void sleep_for (T);
}
}

#include "mcx51x.hpp"

#define concat1(x,y) x ## y
#define concat(x,y) concat1(x,y)
#define test_func concat (test_, __LINE__)

using namespace dev::mcx51x;


// -------------------------------------------------------------------

#if 1
uint32_t test_func (void)
{
  using x = std::ratio_divide < std::ratio<128, 18>, std::ratio<16, 1>>;
  constexpr auto retval = (x::num << 16) | x::den;
  static_assert (retval == 0x40009, "");
  return retval;
}

uint32_t test_func (void)
{
  using x = std::ratio_divide < std::ratio<16, 1>, std::ratio <128, 18> >;
  constexpr auto retval = (x::num << 16) | x::den;
  static_assert (retval == 0x90004, "");
  return retval;
}

uint32_t test_func (void)
{
  using x = std::ratio_divide < std::ratio<128, 18>, std::ratio<16, 1>>;
  using x2 = std::ratio_multiply < x, x >;

  constexpr auto retval = (x2::num << 16) | x2::den;
  static_assert (retval == 0x100051, "");
  return retval;
}

void transform_tests (void)
{
  static_assert (native_to_pps<std::ratio<16'000'000,1>, 10'000'000> (1) == 1, "");
  static_assert (pps_to_native<std::ratio<16'000'000,1>, 10'000'000> (1) == 1, "");

  static_assert (native_to_pps<std::ratio<16'000'000,1>, 10'000'000> (10) == 10, "");
  static_assert (pps_to_native<std::ratio<16'000'000,1>, 10'000'000> (10) == 10, "");

  static_assert (native_to_pps<std::ratio<16'000'000,1>, 10'000'000> (10000) == 10000, "");
  static_assert (pps_to_native<std::ratio<16'000'000,1>, 10'000'000> (10000) == 10000, "");

  static_assert (native_to_pps<std::ratio<12'000'000,1>, 10'000'000> (500) == 375, "");
  static_assert (pps_to_native<std::ratio<12'000'000,1>, 10'000'000> (375) == 500, "");

  static_assert (
	native_to_pps<std::ratio<20'000'000,1>, 1 << 24> (
		pps_to_native<std::ratio<20'000'000,1>, 1 << 24> (8000)) == 8000, "");

  static_assert (
	native_to_pps<std::ratio<12'000'000,1>, 1 << 24> (
		pps_to_native<std::ratio<12'000'000,1>, 1 << 24> (8000)) == 8000, "");


  static_assert (native_to_pps2<std::ratio<16'000'000,1>, 10'000'000> (1) == 1, "");
  static_assert (pps2_to_native<std::ratio<16'000'000,1>, 10'000'000> (1) == 1, "");

  static_assert (native_to_pps2<std::ratio<16'000'000,1>, 10'000'000> (10) == 10, "");
  static_assert (pps2_to_native<std::ratio<16'000'000,1>, 10'000'000> (10) == 10, "");

  static_assert (native_to_pps2<std::ratio<16'000'000,1>, 10'000'000> (10000) == 10000, "");
  static_assert (pps2_to_native<std::ratio<16'000'000,1>, 10'000'000> (10000) == 10000, "");

  static_assert (native_to_pps2<std::ratio<12'000'000,1>, 10'000'000> (500) == 281, "");
  static_assert (pps2_to_native<std::ratio<12'000'000,1>, 10'000'000> (281) == 500, "");

  static_assert (native_to_pps3<std::ratio<12'000'000,1>, 1073741823> (180000000) == 75937500, "");
  static_assert (pps3_to_native<std::ratio<12'000'000,1>, 1073741823> (75937500) == 180000000, "");

}

uint32_t test_func (uint32_t val)
{
  return pps_to_native<std::ratio<12'000'000,1>, 1<<24> (val);
}

uint32_t test_func (uint32_t val)
{
  return native_to_pps3<std::ratio<12'000'000,1>> (val);
}

#endif

// MCX clock = 1 / ((timer counter + 1)*2/PCLKB)
// = std::ratio<PCLKB, (counter+1)*2>


// -------------------------------------------------------------------

#if 1

// RX63 48 MHz PCLKB MCX clocks

static constexpr unsigned int PBCLK48 = 48'000'000;

// timer count = 1 = 12 MHz
uint32_t test_func (uint32_t val)
{
  return native_to_pps<std::ratio<PBCLK48,(1+1)*2>> (val);
}

uint32_t test_func (uint32_t val)
{
  return pps_to_native<std::ratio<PBCLK48,(1+1)*2>> (val);
}

uint32_t test_func (uint32_t val)
{
  return native_to_pps2<std::ratio<PBCLK48,(1+1)*2>> (val);
}

// timer count = 2 = 8 MHz
uint32_t test_func (uint32_t val)
{
  return native_to_pps<std::ratio<PBCLK48,(2+1)*2>> (val);
}

uint32_t test_func (uint32_t val)
{
  return pps_to_native<std::ratio<PBCLK48,(2+1)*2>> (val);
}

uint32_t test_func (uint32_t val)
{
  return native_to_pps2<std::ratio<PBCLK48,(2+1)*2>> (val);
}

// timer count = 3 = 6 MHz
uint32_t test_func (uint32_t val)
{
  return native_to_pps<std::ratio<PBCLK48,(3+1)*2>> (val);
}

uint32_t test_func (uint32_t val)
{
  return pps_to_native<std::ratio<PBCLK48,(3+1)*2>> (val);
}

uint32_t test_func (uint32_t val)
{
  return native_to_pps2<std::ratio<PBCLK48,(3+1)*2>> (val);
}

// timer count = 4 = 4.8 MHz
uint32_t test_func (uint32_t val)
{
  return native_to_pps<std::ratio<PBCLK48,(4+1)*2>> (val);
}

uint32_t test_func (uint32_t val)
{
  return pps_to_native<std::ratio<PBCLK48,(4+1)*2>> (val);
}

uint32_t test_func (uint32_t val)
{
  return native_to_pps2<std::ratio<PBCLK48,(4+1)*2>> (val);
}

// timer count = 5 = 4 MHz
uint32_t test_func (uint32_t val)
{
  return native_to_pps<std::ratio<PBCLK48,(5+1)*2>> (val);
}

uint32_t test_func (uint32_t val)
{
  return pps_to_native<std::ratio<PBCLK48,(5+1)*2>> (val);
}

uint32_t test_func (uint32_t val)
{
  return native_to_pps2<std::ratio<PBCLK48,(5+1)*2>> (val);
}


// timer count = 6 = 3.428571429 MHz
uint32_t test_func (uint32_t val)
{
  return native_to_pps<std::ratio<PBCLK48,(6+1)*2>> (val);
}

uint32_t test_func (uint32_t val)
{
  return pps_to_native<std::ratio<PBCLK48,(6+1)*2>> (val);
}

uint32_t test_func (uint32_t val)
{
  return native_to_pps2<std::ratio<PBCLK48,(6+1)*2>> (val);
}

// timer count = 7 = 3 MHz
uint32_t test_func (uint32_t val)
{
  return native_to_pps<std::ratio<PBCLK48,(7+1)*2>> (val);
}

uint32_t test_func (uint32_t val)
{
  return pps_to_native<std::ratio<PBCLK48,(7+1)*2>> (val);
}

uint32_t test_func (uint32_t val)
{
  return native_to_pps2<std::ratio<PBCLK48,(7+1)*2>> (val);
}

#endif



// -------------------------------------------------------------------
// RX64 / RX71 PCLKB 96 MHz

#if 1
static constexpr unsigned int PBCLK96 = 96'000'000;

// timer count = 1 = 24 MHz
uint32_t test_func (uint32_t val)
{
  return native_to_pps<std::ratio<PBCLK96,(1+1)*2>> (val);
}

uint32_t test_func (uint32_t val)
{
  return pps_to_native<std::ratio<PBCLK96,(1+1)*2>> (val);
}

// timer count = 2 = 16 MHz
uint32_t test_func (uint32_t val)
{
  return native_to_pps<std::ratio<PBCLK96,(2+1)*2>> (val);
}

uint32_t test_func (uint32_t val)
{
  return pps_to_native<std::ratio<PBCLK96,(2+1)*2>> (val);
}

// timer count = 3 = 12 MHz
uint32_t test_func (uint32_t val)
{
  return native_to_pps<std::ratio<PBCLK96,(3+1)*2>> (val);
}

uint32_t test_func (uint32_t val)
{
  return pps_to_native<std::ratio<PBCLK96,(3+1)*2>> (val);
}

// timer count = 4 = 9.6 MHz
uint32_t test_func (uint32_t val)
{
  return native_to_pps<std::ratio<PBCLK96,(4+1)*2>> (val);
}

uint32_t test_func (uint32_t val)
{
  return pps_to_native<std::ratio<PBCLK96,(4+1)*2>> (val);
}

// timer count = 5 = 8 MHz
uint32_t test_func (uint32_t val)
{
  return native_to_pps<std::ratio<PBCLK96,(5+1)*2>> (val);
}

uint32_t test_func (uint32_t val)
{
  return pps_to_native<std::ratio<PBCLK96,(5+1)*2>> (val);
}

// timer count = 6 = 6.857142857 MHz
uint32_t test_func (uint32_t val)
{
  return native_to_pps<std::ratio<PBCLK96,(6+1)*2>> (val);
}

uint32_t test_func (uint32_t val)
{
  return pps_to_native<std::ratio<PBCLK96,(6+1)*2>> (val);
}

// timer count = 7 = 6 MHz
uint32_t test_func (uint32_t val)
{
  return native_to_pps<std::ratio<PBCLK96,(7+1)*2>> (val);
}

uint32_t test_func (uint32_t val)
{
  return pps_to_native<std::ratio<PBCLK96,(7+1)*2>> (val);
}

#endif

// -------------------------------------------------------------------
// RX64 / RX71 PCLKB 120 MHz

#if 1
static constexpr unsigned int PBCLK120 = 120'000'000;

// timer count = 2 = 20 MHz
uint32_t test_func (uint32_t val)
{
  return native_to_pps<std::ratio<PBCLK120,(2+1)*2>> (val);
}

uint32_t test_func (uint32_t val)
{
  return pps_to_native<std::ratio<PBCLK120,(2+1)*2>> (val);
}

// timer count = 3 = 15 MHz
uint32_t test_func (uint32_t val)
{
  return native_to_pps<std::ratio<PBCLK120,(3+1)*2>> (val);
}

uint32_t test_func (uint32_t val)
{
  return pps_to_native<std::ratio<PBCLK120,(3+1)*2>> (val);
}

// timer count = 4 = 12 MHz
uint32_t test_func (uint32_t val)
{
  return native_to_pps<std::ratio<PBCLK120,(4+1)*2>> (val);
}

uint32_t test_func (uint32_t val)
{
  return pps_to_native<std::ratio<PBCLK120,(4+1)*2>> (val);
}

// timer count = 5 = 10 MHz
uint32_t test_func (uint32_t val)
{
  return native_to_pps<std::ratio<PBCLK120,(5+1)*2>> (val);
}

uint32_t test_func (uint32_t val)
{
  return pps_to_native<std::ratio<PBCLK120,(5+1)*2>> (val);
}

// timer count = 6 = 8.571428571 MHz
uint32_t test_func (uint32_t val)
{
  return native_to_pps<std::ratio<PBCLK120,(6+1)*2>> (val);
}

uint32_t test_func (uint32_t val)
{
  return pps_to_native<std::ratio<PBCLK120,(6+1)*2>> (val);
}

// timer count = 7 = 7.5 MHz
uint32_t test_func (uint32_t val)
{
  return native_to_pps<std::ratio<PBCLK120,(7+1)*2>> (val);
}

uint32_t test_func (uint32_t val)
{
  return pps_to_native<std::ratio<PBCLK120,(7+1)*2>> (val);
}

// timer count = 8 = 6.666666667 MHz
uint32_t test_func (uint32_t val)
{
  return native_to_pps<std::ratio<PBCLK120,(8+1)*2>> (val);
}

uint32_t test_func (uint32_t val)
{
  return pps_to_native<std::ratio<PBCLK120,(8+1)*2>> (val);
}

#endif


uint64_t test_func (void)
{
  using a = std::ratio < 48'000'000, (1+1)*2 >;
  using b = std::ratio < 16'000'000, 1 >;

  using pow1 = std::ratio_divide <a, b>;

  return ((uint64_t)pow1::num << 32) | pow1::den;
}

