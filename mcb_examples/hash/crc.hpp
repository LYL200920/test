
/*

generic lookup table based CRC calculator.

the lookup tables are generated at compile time using constexpr functions.
this consumes only ROM and no RAM.  it has no runtime init cost.

the code is based on boost/crc.hpp but has been adapted for C++14.
some other external references:

http://www.boost.org/doc/libs/1_37_0/boost/crc.hpp
http://www.zlib.net/crc_v3.txt
https://en.wikipedia.org/wiki/Cyclic_redundancy_check
http://www.opensource.apple.com/source/xnu/xnu-1456.1.26/bsd/libkern/crc32.c
http://www.zorc.breitbandkatze.de/crctester.c
https://github.com/hazelnusse/crc7/blob/master/crc7.cc

*/

#ifndef includeguard_hash_crc_hpp_includeguard
#define includeguard_hash_crc_hpp_includeguard

#include <cstdint>
#include <cstddef>
#include <type_traits>
#include <array>

#include <utils/rodata.hpp>
#include <utils/bits.hpp>

namespace hash
{

namespace crc_impl_detail
{

// for a specified bit count, select the matching integral type

template < unsigned int BitCount > struct select_int
{
  typedef typename std::conditional < BitCount <= 8, uint8_t,
	  typename std::conditional < BitCount <= 16, uint16_t,
	  typename std::conditional < BitCount <= 32, uint32_t,
	  typename std::conditional < BitCount <= 64, uint64_t,
	  void

	>::type >::type >::type >::type type;
};

template < unsigned int BitCount,
	   typename select_int<BitCount>::type TruncPoly,
	   bool ReflectInput >
constexpr inline std::array < typename select_int<BitCount>::type, 256 >
make_table (void)
{
  typedef typename select_int<BitCount>::type value_type;

  value_type result[256] = { };

  const value_type highbit = value_type (1) << (BitCount-1);

  for (unsigned int dividend = 0; dividend < 256; ++dividend)
  {
    value_type remainder = 0;

    // go through all the dividend's bits
    for (uint8_t mask = 1 << 7; mask != 0; mask >>= 1)
    {
      // check if divisor fits
      if (dividend & mask)
	remainder ^= highbit;

      // do polynominal division
      if (remainder & highbit)
	remainder = (remainder << 1) ^ TruncPoly;
      else
	remainder = remainder << 1;
    }

    // can't use reverse_bits on the array write index.  if bits are reflected,
    // it has to be done with a separate array.
    // see also
    // https://gcc.gnu.org/bugzilla/show_bug.cgi?id=70074
    result[dividend] = ReflectInput
		       ? utils::reverse_bits<BitCount> (remainder) : remainder;
  }

  // https://gcc.gnu.org/bugzilla/show_bug.cgi?id=70074
  if (ReflectInput)
  {
    value_type tmp[256] = { };
    for (unsigned int i = 0; i < 256; ++i)
      tmp[i] = result[i];

    for (unsigned int i = 0; i < 256; ++i)
      result[i] = tmp[ utils::reverse_bits<8> (i) ];
  }

  return utils::make_array (result);
}

// -------------------

template < unsigned int BitCount,
	   typename select_int<BitCount>::type TruncPoly,
	   bool ReflectInput,
	   bool ReflectRemainder >
struct core
{
  typedef typename crc_impl_detail::select_int<BitCount>::type value_type;
  typedef value_type (*process_func_t)(value_type, const uint8_t*, const uint8_t*);

  static constexpr std::array <value_type, 256> table =
			make_table< BitCount, TruncPoly, ReflectInput > ();

  static process_func_t func;

  static constexpr uint8_t table_index (value_type rem, uint8_t x)
  {
    if (ReflectInput)
      return x ^ rem;
    else
      return x ^ (BitCount > 8 ? (rem >> (BitCount - 8))
			       : (rem << (8 - BitCount)));
  }

  static constexpr value_type shift (value_type rem)
  {
    return ReflectInput ? rem >> 8 : rem << 8;
  }

  static value_type
  default_process_bytes (value_type rem, const uint8_t* in, const uint8_t* in_end)
  {
    for (; in != in_end; ++in)
    {
      // Compare the new byte with the remainder's higher bits to
      // get the new bits, shift out the remainder's current higher
      // bits, and update the remainder with the polynominal division
      // of the new bits.
      auto i = table_index (rem, *in);
      rem = table[i] ^ shift (rem);
    }

    return rem;
  }

  static value_type
  process_bytes (value_type rem, const uint8_t* in, const uint8_t* in_end)
  {
   // return func (rem, in, in_end);
    return default_process_bytes (rem, in, in_end);
  }

  constexpr static bool is_remainder_reflected (void) { return false; }
};

// instantiate static table variable.
template < unsigned int BitCount,
	   typename crc_impl_detail::select_int<BitCount>::type TruncPoly,
	   bool ReflectInput, bool ReflectRemainder >
constexpr std::array< typename select_int<BitCount>::type, 256 >
  core<BitCount, TruncPoly, ReflectInput, ReflectRemainder>::table;

template <unsigned int BitCount,
	  typename crc_impl_detail::select_int<BitCount>::type TruncPoly,
	  bool ReflectInput, bool ReflectRemainder >
typename core < BitCount, TruncPoly, ReflectInput, ReflectRemainder >::process_func_t
  core < BitCount, TruncPoly, ReflectInput, ReflectRemainder >::func =
    core < BitCount, TruncPoly, ReflectInput, ReflectRemainder >::default_process_bytes;

} // namespace crc_impl_detail


// ---------------------------------------------------------------------------

template <
  // number of crc result bits (polynomial order in bits)
  unsigned int BitCount,

  // normal polynomial without the leading 1 bit.
  typename crc_impl_detail::select_int<BitCount>::type TruncPoly,

  // initial remainder
  typename crc_impl_detail::select_int<BitCount>::type InitRem = 0,

  // final xor value
  typename crc_impl_detail::select_int<BitCount>::type FinalXor = 0,

  // input data byte reflected before processing (LSB / MSB first)
  bool ReflectInput = false,

  // output CRC reflected before the xor
  bool ReflectRemainder = false >
class crc
{
public:
  // the type of the calculated CRC value.
  typedef typename crc_impl_detail::select_int<BitCount>::type value_type;

  // the type of the input data value(s).
  typedef uint8_t input_value_type;

  static constexpr unsigned int bit_count = BitCount;
  static constexpr value_type truncated_polynomial = TruncPoly;
  static constexpr value_type initial_remainder = InitRem;
  static constexpr value_type final_xor_value = FinalXor;
  static constexpr bool reflect_input = ReflectInput;
  static constexpr bool reflect_remainder = ReflectRemainder;
  static constexpr bool reflect_output = ReflectInput != ReflectRemainder;

  static constexpr value_type value_mask = (((value_type (1) << (BitCount-1))-1) << 1) | 1;

  crc (void) : m_remainder (initial_remainder) { }
  crc (value_type init_rem) : m_remainder (init_rem) { }

  const value_type& remainder (void) const { return m_remainder; }

  value_type operator () (const void* data, std::size_t byte_count)
  {
    m_remainder =
	input_bytes (initial_remainder,
		     (const uint8_t*)data, (const uint8_t*)data + byte_count);
    return get_output ();
  }

  // for appending data to a partial crc
  value_type operator () (const crc& prev,
			  const void* data, std::size_t byte_count)
  {
    m_remainder = input_bytes (prev.remainder (),
			       (const uint8_t*)data, (const uint8_t*)data + byte_count);
    return get_output ();
  }

  template <typename Iterator>
  value_type operator () (Iterator begin, Iterator end)
  {
    // FIXME: specialize for random_iterator.
    // if input_bytes works on a ptr and a count, the code is a little bit better.
    m_remainder = input_bytes (initial_remainder, begin, end);
    return get_output ();
  }

  template <typename Iterator>
  value_type operator () (const crc& prev, Iterator begin, Iterator end)
  {
    m_remainder = input_bytes (prev.remainder (), begin, end);
    return get_output ();
  }

  value_type operator () (void) const
  {
    return get_output ();
  }

  //static const auto& get_table (void) { return table; }

private:
  value_type m_remainder;

  static value_type input_bytes (value_type rem,
				 const uint8_t* in, const uint8_t* in_end)
  {
    return crc_impl_detail::core<BitCount, TruncPoly, ReflectInput, ReflectRemainder>
		::process_bytes (rem, in, in_end);
  }

  value_type get_output (void) const
  {
    return ((reflect_output
	     && crc_impl_detail::core<BitCount, TruncPoly, ReflectInput, ReflectRemainder>
		::is_remainder_reflected () == false
	     ? utils::reverse_bits<BitCount> (m_remainder)
	     : m_remainder)
	    ^ final_xor_value) & value_mask;
  }
};

// ----------------------------------------------------------------------------
// some standard CRC types.
// https://en.wikipedia.org/wiki/Polynomial_representations_of_cyclic_redundancy_checks
// http://reveng.sourceforge.net/crc-catalogue/


// CRC-1 (most hardware; also known as parity bit)
// x + 1
typedef crc < 1, 0x01 > crc_1;

// CRC-3
typedef crc < 3, 0x03, 0x07, 0x00, true, true> crc_3;

// CRC-4-ITU (ITU-T G.704, p. 12)
// x^4 + x + 1
typedef crc < 4, 0x3, 0x00, 0x00, true, true > crc_4_itu;

// CRC-5-EPC (Gen 2 RFID)
// x^5 + x^3 + 1
typedef crc < 5, 0x09, 0x09, 0x00, false, false > crc_5_epc;

// CRC-5-ITU (ITU-T G.704, p. 9)
// x^5 + x^4 + x^2 + 1
typedef crc < 5, 0x15 > crc_5_itu;

// CRC-5-USB (USB token packets)
// x^5 + x^2 + 1
typedef crc < 5, 0x05, 0x1F, 0x1F, true, true > crc_5_usb;

// CRC-6-ITU (ITU-T G.704, p. 3)
// x^6 + x + 1
typedef crc < 6, 0x03, 0x00, 0x00, true, true > crc_6_itu;

// CRC-7 (telecom systems, ITU-T G.707, ITU-T G.832, MMC, SD)
// x^7 + x^3 + 1
typedef crc < 7, 0x09, 0x00, 0x00, false, false > crc_7;

// CRC-8
// x^8 + x^2 + x + 1
typedef crc < 8, 0x07, 0x00, 0x00, false, false > crc_8;

// CRC-8-CCITT (ATM HEC), ISDN Header Error Control and Cell Delineation ITU-T I.432.1 (02/99)
// x^8 + x^2 + x + 1
typedef crc < 8, 0x07, 0x00, 0x55, false, false > crc_8_ccitt;

// CRC-8-Dallas/Maxim (1-Wire bus)
// x^8 + x^5 + x^4 + 1
typedef crc < 8, 0x31, 0x00, 0x00, true, true > crc_8_dallas;

// CRC-8 DVB_S2
// x^8 + x^7 + x^6 + x^4 + x^2 + 1
typedef crc < 8, 0xD5, 0x00, 0x00, false, false > crc_8_dvb_s2;

// CRC-8-SAE J1850
// x^8 + x^4 + x^3 + x^2 + 1
typedef crc < 8, 0x1D, 0xFF, 0xFF, false, false > crc_8_sae;

// CRC-8-WCDMA
// x^8 + x^7 + x^4 + x^3 + x + 1
typedef crc < 8, 0x9B, 0x00, 0x00, true, true > crc_8_wcdma;

// CRC-10 (ATM; ITU-T I.610)
// x^10 + x^9 + x^5 + x^4 + x + 1
typedef crc < 10, 0x233, 0x000, 0x000, false, false > crc_10;

// CRC-11 (FlexRay)
// x^11 + x^9 + x^8 + x^7 + x^2 + 1
typedef crc < 11, 0x385, 0x01A, 0x000, false, false > crc_11;

// CRC-12 (telecom systems)
// x^12 + x^11 + x^3 + x^2 + x + 1
typedef crc < 12, 0x80F, 0x000, 0x000, false, false > crc_12;

// CRC-13-BBC
// x^13 + x^12 + x^11 + x^10 + x^7 + x^6 + x^5 + x^4 + x^2 + 1
typedef  crc < 13, 0x1CF5, 0x0000, 0x0000, false, false > crc_13_bbc;

// CRC-15-CAN
// x^15 + x^14 + x^10 + x^8 + x^7 + x^4 + x^3 + 1
typedef crc < 15, 0x4599, 0x0000, 0x0000, false, false > crc_15_can;

// CRC-16-IBM (Bisync, Modbus, USB, ANSI X3.28, many others; also known as CRC-16 and CRC-16-ANSI)
// x^16 + x^15 + x^2 + 1
typedef crc < 16, 0x8005, 0x0000, 0x0000, true, true > crc_16_ibm;

// CRC-16 (USB)
typedef crc < 16, 0x8005, 0xFFFF, 0xFFFF, true, true > crc_16_usb;

// CRC-16 XMODEM
//typedef crc < 16, 0x8408, 0x0000, 0x0000, true, true > crc_16_xmodem;
typedef crc < 16, 0x1021, 0x0000, 0x0000, false, false > crc_16_xmodem;
typedef crc_16_xmodem crc_16_zmodem;

// CRC-16-CCITT (X.25, V.41, HDLC, XMODEM, Bluetooth, SD, many others; known as CRC-CCITT)
// x^16 + x^12 + x^5 + 1
typedef crc < 16, 0x1021, 0xFFFF, 0, false, false > crc_16_ccitt;

// CRC-16-T10-DIF (SCSI DIF)
// x^16 + x^15 + x^11 + x^9 + x^8 + x^7 + x^5 + x^4 + x^2 + x + 1
typedef crc < 16, 0x8BB7, 0x0000, 0x0000, false, false > crc_16_t10_dif;

// CRC-16-DNP (DNP, IEC 870, M-Bus)
// x^16 + x^13 + x^12 + x^11 + x^10 + x^8 + x^6 + x^5 + x^2 + 1
typedef crc < 16, 0x3D65, 0x0000, 0xFFFF, true, true > crc_16_dnp;

// CRC-16-DECT (cordless telephones)
// x^16 + x^10 + x^8 + x^7 + x^3 + 1
typedef crc < 16, 0x0589, 0x0000, 0x0001, false, false > crc_16_dect_r;
typedef crc < 16, 0x0589, 0x0000, 0x0000, false, false > crc_16_dect_x;


// CRC-24 (FlexRay)
// x^24 + x^22 + x^20 + x^19 + x^18 + x^16 + x^14 + x^13 + x^11 + x^10 + x^8 + x^7 + x^6 + x^3 + x + 1
typedef crc < 24, 0x5D6DCB, 0xFEDCBA, 0x000000, false, false > crc_24_flexray;

// CRC-24-Radix-64 (OpenPGP)
// x^24 + x^23 + x^18 + x^17 + x^14 + x^11 + x^10 + x^7 + x^6 + x^5 + x^4 + x^3 + x + 1
typedef crc < 24, 0x864CFB, 0xB704CE, 0x000000, false, false > crc_24_radix_64;

// CRC-30 (CDMA)
// x^30 + x^29 + x^21 + x^20 + x^15 + x^13 + x^12 + x^11 + x^8 + x^7 + x^6 + x^2 + x + 1
typedef crc < 30, 0x2030B9C7, 0x3FFFFFFF, 0x3FFFFFFF, false, false > crc_30;

// CRC-32 (ISO 3309, ANSI X3.66, FIPS PUB 71, FED-STD-1003, ITU-T V.42, Ethernet, SATA, MPEG-2, Gzip, PKZIP, POSIX cksum, PNG, ZMODEM)
// x^32 + x^26 + x^23 + x^22 + x^16 + x^12 + x^11 + x^10 + x^8 + x^7 + x^5 + x^4 + x^2 + x + 1
typedef crc < 32, 0x04C11DB7, 0xFFFFFFFF, 0xFFFFFFFF, true, true > crc_32;

typedef crc < 32, 0x04C11DB7, 0x7FFFFFFF, 0x7FFFFFFF, false, false > crc_32_mpeg2;
typedef crc < 32, 0x04C11db7, 0x00000000, 0xFFFFFFFF, false, false > crc_32_posix;

// CRC-32C (Castagnoli) (iSCSI & SCTP, G.hn payload, SSE4.2)
// x^32 + x^28 + x^27 + x^26 + x^25 + x^23 + x^22 + x^20 + x^19 + x^18 + x^14 + x^13 + x^11 + x^10 + x^9 + x^8 + x^6 + 1
typedef crc < 32, 0x1EDC6F41, 0xFFFFFFFF, 0xFFFFFFFF, true, true > crc_32c;

// CRC-32Q (aviation; AIXM)
// x^32 + x^31 + x^24 + x^22 + x^16 + x^14 + x^8 + x^7 + x^5 + x^3 + x + 1
typedef crc < 32, 0x814141AB, 0x00000000, 0x00000000, false, false > crc_32q;

// CRC-40-GSM (GSM control channel)
// x^40 + x^26 + x^23 + x^17 + x^3 + 1
typedef crc < 40, 0x0004820009LL, 0x0000000000LL, 0xFFFFFFFFFFLL, false, false > crc_40;

// CRC-64-ECMA-182 (as described in ECMA-182 p. 51)
// x^64 + x^62 + x^57 + x^55 + x^54 + x^53 + x^52 + x^47 + x^46 + x^45 + x^40 + x^39 + x^38 + x^37 + x^35 + x^33 + x^32 + x^31 + x^29 + x^27 + x^24 + x^23 + x^22 + x^21 + x^19 + x^17 + x^13 + x^12 + x^10 + x^9 + x^7 + x^4 + x + 1
typedef crc < 64, 0x42F0E1EBA9EA3693LL, 0x0000000000000000LL, 0x0000000000000000LL, false, false > crc_64_ecma_182;

} // namespace hash

#endif // includeguard_hash_crc_hpp_includeguard
