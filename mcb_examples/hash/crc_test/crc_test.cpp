
#include <iostream>
#include <cstdio>
#include <iomanip>

#include "MCBcode/hash/crc.hpp"

namespace test_00
{

void make_crc_table (unsigned long crcTable[])
{
    unsigned long POLYNOMIAL = 0xEDB88320;
    unsigned long remainder;
    unsigned char b = 0;
    do{
        // Start with the data byte
        remainder = b;
        for (unsigned long bit = 8; bit > 0; --bit)
        {
            if (remainder & 1)
                remainder = (remainder >> 1) ^ POLYNOMIAL;
            else
                remainder = (remainder >> 1);
        }
        crcTable[(size_t)b] = remainder;
    } while(0 != ++b);
}

unsigned long gen_crc(unsigned char *p, size_t n, unsigned long crcTable[])
{
unsigned long crc = 0xfffffffful;
size_t i;
    for(i = 0; i < n; i++)
        crc = crcTable[*p++ ^ (crc&0xff)] ^ (crc>>8);
    return(~crc);
}

int main (void)
{
  unsigned long crcTable[256];
  make_crc_table (crcTable);

  // Print the CRC table
  for (size_t i = 0; i < 256; i++)
  {
      std::cout << std::setfill('0') << std::setw(8)
                << std::hex << crcTable[i];
      if (i % 16 == 15)
        std::cout << std::endl;
      else
        std::cout << ", ";
  }

  uint8_t test_data[64] = { 0 };
  std::cout << "test crc = "
	    << std::setfill ('0') << std::setw (8)
	    << std::hex << (uint32_t)gen_crc (test_data, 64, crcTable)
	    << std::endl;

  return 0;
}

} // test_00



// ---------------------------------------------------------------------------

namespace test_01
{

unsigned long gen_crc (unsigned char *p, size_t n)
{
  hash::crc < 32, 0x04C11DB7, 0xFFFFFFFF, 0xFFFFFFFF, true, true > crc;
  return crc (p, n);
}

int main (void)
{
  hash::crc < 32, 0x04C11DB7, 0xFFFFFFFF, 0xFFFFFFFF, true, true > crc;

  for (size_t i = 0; i < 256; i++)
  {
    std::cout << std::setfill('0') << std::setw(8)
              << std::hex << crc.get_table ()[i];
    if (i % 16 == 15)
      std::cout << std::endl;
    else
      std::cout << ", ";
  }

  uint8_t test_data[64] = { 0 };
  std::cout << "test crc = "
	    << std::setfill ('0') << std::setw (8)
	    << std::hex << (uint32_t)gen_crc (test_data, 64)
	    << std::endl;

  return 0;
}

} // test_01

// ---------------------------------------------------------------------------


#include "boost_crc.hpp"

namespace test_02
{

typedef boost::crc_optimal<32, 0x04C11DB7, 0xFFFFFFFF, 0xFFFFFFFF, true, true> crc_t;
//typedef boost::crc_32_type crc_t;

//typedef boost::crc_optimal<32, 0x04C11DB7, 0xFFFFFFFF, 0xFFFFFFFF, false, false> crc_t;


uint32_t gen_crc (unsigned char* p, size_t n)
{
  crc_t crc;

  crc.process_bytes (p, n);
  return crc ();
}

int main (void)
{
  crc_t::crc_table_type::init_table ();

  for (size_t i = 0; i < 256; i++)
  {
    std::cout << std::setfill('0') << std::setw(8)
              << std::hex << crc_t::crc_table_type::table_[i];
    if (i % 16 == 15)
      std::cout << std::endl;
    else
      std::cout << ", ";
  }

  uint8_t test_data[64] = { 0 };
  std::cout << "test crc = "
	    << std::setfill ('0') << std::setw (8)
	    << std::hex << (uint32_t)gen_crc (test_data, 64)
	    << std::endl;
  return 0;
}



} // test_02


// ---------------------------------------------------------------------------

namespace test_03
{

static uint8_t CRCTable[256];

void GenerateCRCTable()
{
  int i, j;
  uint8_t CRCPoly = 0x89;  // the value of our CRC-7 polynomial
 
  // generate a table value for all 256 possible byte values

  for (i = 0; i < 256; ++i)
  {
    CRCTable[i] = (i & 0x80) ? i ^ CRCPoly : i;

    for (j = 1; j < 8; ++j)
    {
        CRCTable[i] <<= 1;
        if (CRCTable[i] & 0x80)
            CRCTable[i] ^= CRCPoly;
    }
  }
}

uint8_t getCRC (uint8_t message[], int length)
{
  int i;
  uint8_t CRC = 0;

  for (i = 0; i < length; ++i)
    CRC = CRCTable[(CRC << 1) ^ message[i]];

  return CRC;
}


int main (void)
{
  GenerateCRCTable ();

  for (size_t i = 0; i < 256; i++)
  {
    std::cout << std::setfill('0') << std::setw(2)
              << std::hex << (uint32_t)CRCTable[i];
    if (i % 16 == 15)
      std::cout << std::endl;
    else
      std::cout << ", ";
  }

  uint8_t test_data[64];
  for (unsigned int i = 0; i < 64; ++i)
    test_data[i] = i*2 + 123;

  std::cout << "test crc = "
	    << std::setfill ('0') << std::setw (8)
	    << std::hex << (uint32_t)getCRC (test_data, 64)
	    << std::endl;
  return 0;
}

} // test_03

// ---------------------------------------------------------------------------

namespace test_04
{
typedef hash::crc < 7, 0x09, 0, 0, false, false > crc_t;

unsigned long gen_crc (unsigned char *p, size_t n)
{
  crc_t crc;
  return crc (p, n);
}

int main (void)
{
  crc_t crc;

  for (size_t i = 0; i < 256; i++)
  {
    std::cout << std::setfill('0') << std::setw(2)
              << std::hex << (uint32_t)crc.get_table ()[i];
    if (i % 16 == 15)
      std::cout << std::endl;
    else
      std::cout << ", ";
  }
/*
  uint8_t test_data[64];
  for (unsigned int i = 0; i < 64; ++i)
    test_data[i] = i*2 + 123;
*/
  uint8_t test_data[] = { 0x37, 0x00, 0x00, 0x01, 0x20 };

  std::cout << "test crc = "
	    << std::setfill ('0') << std::setw (8)
	    << std::hex << (uint32_t)gen_crc (test_data, std::extent<decltype (test_data)>::value)
	    << std::endl;

  return 0;
}



}

// ---------------------------------------------------------------------------


// ---------------------------------------------------------------------------




int main (void)
{
//  std::cout << "test_00" << std::endl;
//  test_00::main ();

//  std::cout << "test_01" << std::endl;
//  test_01::main ();

//  std::cout << "test_02" << std::endl;
//  test_02::main ();


  std::cout << "test_03" << std::endl;
  test_03::main ();

  std::cout << "test_04" << std::endl;
  test_04::main ();

  return 0;
}




#if 0

int main (void)
{
  hash::crc < 32, 0x04C11DB7, 0xFFFFFFFF, 0xFFFFFFFF, true, true> c0;

#if 0
  for (int i = 0; i < 256/4; ++i)
/*
    std::printf ("0x%08x, 0x%08x, 0x%08x, 0x%08x,\n",
		 hash::crc_impl_detail::reflect_bits<32> ( decltype (c0)::get_table ()[i * 4 + 0] ),
		 hash::crc_impl_detail::reflect_bits<32> ( decltype (c0)::get_table ()[i * 4 + 1] ),
		 hash::crc_impl_detail::reflect_bits<32> ( decltype (c0)::get_table ()[i * 4 + 2] ),
		 hash::crc_impl_detail::reflect_bits<32> ( decltype (c0)::get_table ()[i * 4 + 3]) );
*/
    std::printf ("0x%08x, 0x%08x, 0x%08x, 0x%08x,\n",
		  ( decltype (c0)::get_table ()[i * 4 + 0] ),
		  ( decltype (c0)::get_table ()[i * 4 + 1] ),
		  ( decltype (c0)::get_table ()[i * 4 + 2] ),
		  ( decltype (c0)::get_table ()[i * 4 + 3]) );

#endif

  


  return 0;
}

uint32_t test (const void* in)
{
  hash::crc < 32, 0x04C11DB7, 0xFFFFFFFF, 0xFFFFFFFF, true, true> c0;

  return c0 (in, 256);
}

#endif

