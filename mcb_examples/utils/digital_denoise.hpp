/*

digital (binary) de-noise filter

record bits to determine the current state of the filtered signal.


___‾‾___________‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾________
                
  noise         |   valid high     |


0001100100000000111111011111010111110000000000000000

___‾‾__‾________‾‾‾‾‾‾_‾‾‾‾‾_‾_‾‾‾‾‾________________
                
   noise        |  valid high      |  valid low
                  (with noise)


if the output is in a low state it should take X number of high samples within
a time window in order to switch the output state to high.  and vice versa
when the output is a high state.


typically this is implemented in digital-input hardware by shift registers
and by counting bits.

use the last n bits and count the number of zero/one bits.
if there are significantly more non-zero bits than zero bits, the output
signal is considered to be active.

because of the bitshifting approach, there is an inherent delay in the output
signal.  this can be actually useful in some cases.
*/

#ifndef includeguard_utils_digital_denoise_hpp_includeguard
#define includeguard_utils_digital_denoise_hpp_includeguard

#include <bitset>
#include <algorithm>

namespace utils
{

template <
  std::size_t PositiveThreshold,
  std::size_t NegativeThreshold = PositiveThreshold >
class digital_denoise
{
public:
  void clear (void)
  {
    m_output_state = false;
    m_pos_count = 0;
    m_buffer.reset ();
  }

  // returns the current output state
  bool value (void) const
  {
    return m_output_state;
  }

  // inputs one sampled value
  void push_back (bool val)
  {
    m_pos_count += val;
    m_pos_count -= m_buffer[m_buffer.size () - 1];

    m_buffer <<= 1;
    m_buffer[0] = val;

    const auto neg_count = m_buffer.size () - m_pos_count;

    if (m_output_state == false && m_pos_count > PositiveThreshold)
      m_output_state = true;

    else if (m_output_state == true && neg_count > NegativeThreshold)
      m_output_state = false;
  }

private:
  bool m_output_state = false;
  std::size_t m_pos_count = 0;
  std::bitset<std::max (PositiveThreshold, NegativeThreshold) + 1> m_buffer;

};


} // namespace utils
#endif // includeguard_utils_digital_denoise_hpp_includeguard
